// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm VIDC 1080p Video Codec driver
 *
 * Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
 * Copyright (c) 2024, Linux-SHR Project
 *
 * This driver supports the VIDC 1080p video codec found in MSM8660/APQ8060
 * SoCs. Unlike newer Venus cores, VIDC 1080p uses direct register-based
 * HOST2RISC/RISC2HOST command interface.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/interconnect.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>

#include "vidc_core.h"
#include "vidc_dec.h"
#include "vidc_enc.h"

#define VIDC_FW_NAME		"qcom/vidc_1080p.fw"
/*
 * VIDC firmware blob size cap. The legacy Palm webOS-shipped
 * vidc_1080p.fw is 605428 bytes (~592 KB); the previous 512 KB cap
 * was set before the actual blob size was checked. Cap at 1 MB —
 * the firmware-adjacent DMA buffer sizes from this value so we want
 * a sane upper bound, not "open-ended whatever is in /lib/firmware".
 */
#define VIDC_FW_SIZE_MAX	(1024 * 1024)

#define VIDC_INIT_CH_INST_ID	0x0000ffff

/* Interconnect bandwidth for 1080p video (in bytes/sec) */
#define VIDC_BW_AVG		(245 * 1024 * 1024)	/* 245 MB/s average */
#define VIDC_BW_PEAK		(500 * 1024 * 1024)	/* 500 MB/s peak */

/* Clock rates in Hz */
static const unsigned long vidc_clk_rates[] = {
	27000000,
	48000000,
	96000000,
	133330000,
	200000000,
};

static int vidc_clk_enable(struct vidc_core *core)
{
	int ret;

	ret = clk_prepare_enable(core->iface_clk);
	if (ret) {
		dev_err(core->dev, "failed to enable iface clock: %d\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(core->core_clk);
	if (ret) {
		dev_err(core->dev, "failed to enable core clock: %d\n", ret);
		goto err_iface_clk;
	}

	ret = clk_prepare_enable(core->axi_clk);
	if (ret) {
		dev_err(core->dev, "failed to enable axi clock: %d\n", ret);
		goto err_core_clk;
	}

	printk(KERN_EMERG "VIDC: clk_enable: core_clk=%lu Hz iface_clk=%lu Hz axi_clk=%lu Hz\n",
	       clk_get_rate(core->core_clk),
	       clk_get_rate(core->iface_clk),
	       clk_get_rate(core->axi_clk));

	return 0;

err_core_clk:
	clk_disable_unprepare(core->core_clk);
err_iface_clk:
	clk_disable_unprepare(core->iface_clk);
	return ret;
}

static void vidc_clk_disable(struct vidc_core *core)
{
	clk_disable_unprepare(core->axi_clk);
	clk_disable_unprepare(core->core_clk);
	clk_disable_unprepare(core->iface_clk);
}

/*
 * Bring the VIDC core out of reset and arm the firmware boot.
 *
 * Mirrors legacy webOS DDL bring-up sequence
 * (webos-linux-kernel-touchpad/drivers/video/msm/vidc/1080p/ddl/
 * vcd_ddl_vidc.c:30 ddl_vidc_core_init):
 *
 *   1. SW_RESET stage 1: progressively assert resets on VI, RISC,
 *      VIDCCORE+DMX. msleep(1) between stage 1 and stage 2 (legacy
 *      DDL_SW_RESET_SLEEP).
 *   2. SW_RESET stage 2: full RESET_ALL then release everything
 *      except RISC. RISC stays held in reset across the DRAM_BASE
 *      programming so it doesn't speculatively fetch from
 *      DRAM_BASE = 0 (stale value from before our write).
 *   3. Program DRAM_BASE_A/B with the 128 KB-aligned firmware
 *      physical address (caller pre-shifts by 17). Hardware field
 *      is bits [31:17] of the register.
 *   4. AXI halt + reset + burst config + per-channel inst ID init
 *      + clear pending cmd/response registers.
 *   5. Release SW_RESET (RESET_NONE). This is the point the RISC
 *      begins executing the firmware from DRAM_BASE_A; once the
 *      firmware boot stub initialises, it fires an unsolicited
 *      RISC2HOST FW_STATUS_RET IRQ which vidc_boot_firmware()
 *      waits on before issuing SYS_INIT.
 */
int vidc_hw_reset(struct vidc_core *core, u32 dram_base_addr)
{
	u32 axi_status, sw_reset;
	int timeout = 100;

	/*
	 * Stage 1: Progressive assert of VI / RISC / VIDCCORE / DMX into
	 * reset. Use read-modify-write so we walk down from whatever state
	 * the GDSC left us in (the gdsc-qcom ved entry sets
	 * LEGACY_FOOTSWITCH | SW_RESET and pulses VCODEC_AHB_RESET on
	 * enable, which may leave VIDC_REG_SW_RESET at a value other than
	 * the 0x3ff "all released" default). Legacy webOS DDL does the
	 * same RMW (see webos-linux-kernel-touchpad/drivers/video/msm/vidc/
	 * 1080p/ddl/vidc.c:84 — VIDC_HWIO_IN then progressively clear
	 * bits).
	 *
	 * Why RMW matters: hardcoded 0x3f7 / 0x3f6 / 0x3e2 unconditionally
	 * RELEASES blocks that the GDSC's pulse just held in reset. The
	 * second write (transition releasing only-VI-in-reset →
	 * VI+RISC-in-reset) wedged the AHB on the actual hardware; the
	 * post-write dsb never drained. Read-modify-write makes the writes
	 * structural no-ops when the GDSC already put everything in reset
	 * — same behaviour as legacy, no surprise transitions.
	 */
	sw_reset = vidc_read(core, VIDC_REG_SW_RESET);
	sw_reset &= ~VIDC_RESET_VI;
	vidc_write(core, VIDC_REG_SW_RESET, sw_reset);
	sw_reset &= ~VIDC_RESET_RISC;
	vidc_write(core, VIDC_REG_SW_RESET, sw_reset);
	sw_reset &= ~(VIDC_RESET_VIDCCORE | VIDC_RESET_DMX);
	vidc_write(core, VIDC_REG_SW_RESET, sw_reset);

	msleep(1);

	/*
	 * Stage 2: Full reset, then release everything except RISC.
	 *
	 * The second write here mirrors legacy webOS DDL's
	 * VIDC_1080P_RESET_RISC = 0x3fe (i.e. VIDC_RESET_NONE & ~BIT(0)):
	 * all blocks released, *only* the RISC core held in reset across
	 * the DRAM_BASE programming that follows. Writing the single-bit
	 * mask VIDC_RESET_RISC (= BIT(0)) here would do the OPPOSITE —
	 * hold every block in reset *except* RISC, including VIDCCORE +
	 * COMMON which gate the AHB slave interface — and the very next
	 * writel() hangs in arm_heavy_mb (dsb waiting for the store to
	 * drain on a now-dead bus). That was the symptom of the original
	 * mainline bug fixed here.
	 */
	vidc_write(core, VIDC_REG_SW_RESET, VIDC_RESET_ALL);
	vidc_write(core, VIDC_REG_SW_RESET,
		   VIDC_RESET_NONE & ~VIDC_RESET_RISC);

	/*
	 * Program DRAM_BASE_A/B while the RISC is held in reset, so the
	 * first instruction fetch after RESET_NONE points at the firmware
	 * we just memcpy'd into the coherent buffer.
	 *
	 * Write the FULL physical address (not shifted). The hardware
	 * register layout uses bits 31:17 for the address (mask 0xfffe0000),
	 * implying 17-bit alignment, but webOS writes the full address and
	 * the hardware handles it correctly.
	 *
	 * Note: These registers appear to be write-only; readback always
	 * returns 0x00000000 even after writing. This matches webOS kernel
	 * behavior which never reads these registers back.
	 */
	printk(KERN_EMERG "VIDC: hw_reset: writing DRAM_BASE=0x%08x to offsets 0x%03x/0x%03x\n",
	       dram_base_addr, VIDC_REG_DRAM_BASE_A, VIDC_REG_DRAM_BASE_B);
	vidc_write(core, VIDC_REG_DRAM_BASE_A, dram_base_addr);
	vidc_write(core, VIDC_REG_DRAM_BASE_B, dram_base_addr);
	printk(KERN_EMERG "VIDC: hw_reset: readback DRAM_BASE_A=0x%08x DRAM_BASE_B=0x%08x\n",
	       vidc_read(core, VIDC_REG_DRAM_BASE_A),
	       vidc_read(core, VIDC_REG_DRAM_BASE_B));

	/*
	 * Clear RETURNED_CH_INST_ID early, matching webOS sequence.
	 * WebOS calls vidc_1080p_clear_returned_channel_inst_id() right after
	 * DRAM_BASE init and before releasing RISC. This register is used by
	 * firmware to return channel IDs after OPEN_CH commands.
	 */
	printk(KERN_EMERG "VIDC: hw_reset: clearing RETURNED_CH_INST_ID\n");
	vidc_write(core, VIDC_REG_RETURNED_CH_INST_ID, VIDC_INIT_CH_INST_ID);

	/*
	 * WebOS kernel has significant delay between DRAM_BASE write and
	 * AXI halt - it sets up 24 function pointers first. Add a small
	 * delay here to let hardware settle after DRAM_BASE programming.
	 */
	msleep(10);

	/* Halt AXI */
	printk(KERN_EMERG "VIDC: hw_reset: about to halt AXI\n");
	vidc_write(core, VIDC_REG_AXI_CTRL, VIDC_AXI_HALT_REQ);
	printk(KERN_EMERG "VIDC: hw_reset: AXI halt request sent\n");

	/* Wait for AXI halt acknowledgment */
	printk(KERN_EMERG "VIDC: hw_reset: polling for AXI halt ack (need 0x3)\n");
	do {
		axi_status = vidc_read(core, VIDC_REG_AXI_STATUS);
		if (timeout % 50 == 0)  /* Log every ~5ms */
			printk(KERN_EMERG "VIDC: hw_reset: AXI_STATUS=0x%08x\n", axi_status);
		axi_status = axi_status & VIDC_AXI_HALT_ACK_MASK;  /* Bits 1:0, no shift */
		if (axi_status == 0x3)
			break;
		usleep_range(100, 200);
	} while (--timeout > 0);

	if (timeout == 0) {
		printk(KERN_EMERG "VIDC: hw_reset: AXI halt timeout, final status=0x%08x\n",
		       vidc_read(core, VIDC_REG_AXI_STATUS));
		dev_err(core->dev, "AXI halt timeout\n");
		return -ETIMEDOUT;
	}
	printk(KERN_EMERG "VIDC: hw_reset: AXI halt ack received\n");

	/* Reset AXI */
	printk(KERN_EMERG "VIDC: hw_reset: about to reset AXI\n");
	vidc_write(core, VIDC_REG_AXI_CTRL, VIDC_AXI_RESET);
	printk(KERN_EMERG "VIDC: hw_reset: AXI reset bit set\n");
	vidc_write(core, VIDC_REG_AXI_CTRL, 0);
	printk(KERN_EMERG "VIDC: hw_reset: AXI reset cleared\n");

	/* Configure burst sizes */
	printk(KERN_EMERG "VIDC: hw_reset: about to configure burst sizes\n");
	vidc_write(core, VIDC_REG_BURST_CONFIG, (8 << 8) | 8);
	printk(KERN_EMERG "VIDC: hw_reset: burst config done\n");

	/* Initialize channel instance IDs */
	vidc_write(core, VIDC_REG_CH0_INST_ID, VIDC_INIT_CH_INST_ID);
	vidc_write(core, VIDC_REG_CH1_INST_ID, VIDC_INIT_CH_INST_ID);

	/* Clear command registers */
	vidc_write(core, VIDC_REG_RISC2HOST_CMD, VIDC_RESP_EMPTY);
	vidc_write(core, VIDC_REG_HOST2RISC_CMD, VIDC_CMD_EMPTY);

	/* Release reset */
	vidc_write(core, VIDC_REG_SW_RESET, VIDC_RESET_NONE);

	dev_info(core->dev, "hw_reset: released RISC from reset\n");

	/* Sample AXI_STATUS immediately and after short delays to detect RISC instruction fetches */
	{
		int i;
		u32 axi_st, sw_rst, fwver;

		for (i = 0; i < 5; i++) {
			axi_st = vidc_read(core, VIDC_REG_AXI_STATUS);
			sw_rst = vidc_read(core, VIDC_REG_SW_RESET);
			fwver  = vidc_read(core, VIDC_REG_FW_VERSION);
			printk(KERN_EMERG "VIDC: post-release[%d]: AXI_STATUS=0x%08x SW_RESET=0x%08x FW_VERSION=0x%08x\n",
			       i, axi_st, sw_rst, fwver);
			usleep_range(2000, 2100);
		}
	}

	/* Give firmware CPU time to boot and initialize */
	msleep(10);

	return 0;
}

int vidc_send_cmd(struct vidc_core *core, u32 cmd, u32 arg1, u32 arg2,
		  u32 arg3, u32 arg4)
{
	vidc_write(core, VIDC_REG_HOST2RISC_CMD, VIDC_CMD_EMPTY);
	vidc_write(core, VIDC_REG_HOST2RISC_ARG1, arg1);
	vidc_write(core, VIDC_REG_HOST2RISC_ARG2, arg2);
	vidc_write(core, VIDC_REG_HOST2RISC_ARG3, arg3);
	vidc_write(core, VIDC_REG_HOST2RISC_ARG4, arg4);
	vidc_write(core, VIDC_REG_HOST2RISC_CMD, cmd);

	return 0;
}

int vidc_get_response(struct vidc_core *core, u32 *cmd, u32 *arg1,
		      u32 *arg2, u32 *arg3, u32 *arg4)
{
	*cmd = vidc_read(core, VIDC_REG_RISC2HOST_CMD);
	*arg1 = vidc_read(core, VIDC_REG_RISC2HOST_ARG1);
	*arg2 = vidc_read(core, VIDC_REG_RISC2HOST_ARG2);
	*arg3 = vidc_read(core, VIDC_REG_RISC2HOST_ARG3);
	*arg4 = vidc_read(core, VIDC_REG_RISC2HOST_ARG4);

	return 0;
}

static void vidc_clear_interrupt(struct vidc_core *core)
{
	vidc_write(core, VIDC_REG_INTERRUPT, 0);
}

static void vidc_handle_frame_done(struct vidc_core *core,
				   struct vidc_inst *inst)
{
	u32 status;

	/*
	 * Read decoded frame DPB-slot addresses. These are fw-relative
	 * offsets shifted right by VIDC_ADDR_SHIFT — same encoding the
	 * host used when programming DPB_LUMA_BASE / DPB_CHROMA_BASE in
	 * vidc_init_buffers(). The device_run thread will reverse the
	 * encoding to pick the DPB slot to copy out of.
	 */
	inst->display_y_raw = vidc_read(core, VIDC_REG_DEC_DISPLAY_Y);
	inst->display_c_raw = vidc_read(core, VIDC_REG_DEC_DISPLAY_C);

	/*
	 * Display-status disambiguates the four cases the firmware can
	 * report on a FRAME_DONE event:
	 *
	 *   DECODE_AND_DISPLAY — common low-latency path: frame decoded
	 *                        and ready to emit to userspace right now
	 *   DECODE_ONLY        — a B-frame was decoded but is being held
	 *                        in the DPB for reorder; will emerge as a
	 *                        later DISPLAY_ONLY / DECODE_AND_DISPLAY
	 *   DISPLAY_ONLY       — no fresh source consumed; an earlier
	 *                        decode-only frame is now ready to emit
	 *                        (typical during EOS drain)
	 *   DPB_EMPTY          — no more frames to emit (EOS done)
	 *
	 * Field encoding mirrors legacy VIDC_1080P_SI_RG7_DISPLAY_STATUS:
	 * low nibble of VIDC_REG_DEC_DISPLAY_STATUS.
	 */
	status = vidc_read(core, VIDC_REG_DEC_DISPLAY_STATUS);
	inst->display_status = status & VIDC_DISPLAY_STATUS_MASK;

	dev_dbg(core->dev,
		"Frame done: Y_raw=0x%x C_raw=0x%x (offsets 0x%x / 0x%x) status=%u\n",
		inst->display_y_raw, inst->display_c_raw,
		inst->display_y_raw << VIDC_ADDR_SHIFT,
		inst->display_c_raw << VIDC_ADDR_SHIFT,
		inst->display_status);

	/* Read the decoded compressed-frame consumed size */
	inst->result_size = vidc_read(core, VIDC_REG_SEQ_FRAME_SIZE);
}

static void vidc_handle_enc_complete(struct vidc_core *core,
				     struct vidc_inst *inst)
{
	/* Read encoded frame size */
	inst->result_size = vidc_read(core, VIDC_REG_ENC_FRAME_SIZE);

	dev_dbg(core->dev, "Encode complete: size=%u\n", inst->result_size);
}

static void vidc_handle_seq_done(struct vidc_core *core,
				 struct vidc_inst *inst)
{
	/* Read sequence header info */
	inst->seq_height = vidc_read(core, VIDC_REG_SEQ_IMG_SIZE_Y);
	inst->seq_width = vidc_read(core, VIDC_REG_SEQ_IMG_SIZE_X);
	inst->min_dpb_count = vidc_read(core, VIDC_REG_SEQ_MIN_DPB);

	dev_dbg(core->dev, "Sequence done: %ux%u, min_dpb=%u\n",
		inst->seq_width, inst->seq_height, inst->min_dpb_count);

	inst->state = VIDC_STATE_SEQ_PARSED;
}

static irqreturn_t vidc_isr(int irq, void *data)
{
	struct vidc_core *core = data;
	struct vidc_inst *inst;
	u32 cmd, arg1, arg2, arg3, arg4;
	unsigned long flags;
	static DEFINE_RATELIMIT_STATE(rs, HZ, 5);

	spin_lock_irqsave(&core->irqlock, flags);

	/*
	 * Legacy webOS DDL (vcd_ddl_interrupt_handler.c:
	 * ddl_read_and_clear_interrupt) ACKs the IRQ in this specific
	 * order:
	 *   1. Read RISC2HOST_CMD + args   (snapshot the message)
	 *   2. Clear RISC2HOST_CMD         (write EMPTY → message ack)
	 *   3. Clear VIDC_REG_INTERRUPT    (write 0 → IRQ status ack)
	 *
	 * Doing the interrupt-status clear BEFORE the response-register
	 * clear causes a spurious-IRQ storm: the hardware sees the
	 * response register still non-empty when the IRQ-status write
	 * lands, immediately re-asserts the IRQ, and the handler spins
	 * (~164 k cb/sec observed). Match legacy order exactly.
	 */
	vidc_get_response(core, &cmd, &arg1, &arg2, &arg3, &arg4);
	vidc_write(core, VIDC_REG_RISC2HOST_CMD, VIDC_RESP_EMPTY);
	vidc_clear_interrupt(core);

	inst = core->curr_inst;

	if (__ratelimit(&rs))
		dev_info(core->dev,
			 "VIDC IRQ: cmd=%u arg1=0x%x arg2=0x%x inst=%p\n",
			 cmd, arg1, arg2, inst);

	/*
	 * Storm guard: if firmware boot has stalled, the hardware can
	 * re-assert the IRQ line indefinitely with RISC2HOST_CMD =
	 * VIDC_RESP_EMPTY. The handler then re-enters at ~165 k/s and
	 * starves the rest of the system. After a streak of empty IRQs
	 * the chip is wedged regardless — disable the line so the
	 * kernel stays alive and userspace can time out cleanly.
	 */
	if (cmd == VIDC_RESP_EMPTY) {
		if (++core->empty_irq_streak >= 64 &&
		    !core->irq_disabled_by_storm) {
			core->irq_disabled_by_storm = true;
			disable_irq_nosync(core->irq);
			dev_err(core->dev,
				"VIDC IRQ storm (>=64 empty IRQs) — disabling IRQ %d; reboot required to recover\n",
				core->irq);
		}
	} else {
		core->empty_irq_streak = 0;
	}

	switch (cmd) {
	case VIDC_RESP_FW_STATUS:
		/*
		 * Unsolicited "RISC is alive" announcement raised once the
		 * boot stub finishes its DMA-init handshake after we
		 * released SW_RESET. vidc_boot_firmware() blocks on this
		 * completion before sending SYS_INIT — the firmware will
		 * silently drop a SYS_INIT that arrives ahead of its own
		 * FW_STATUS_RET, which is the failure mode the mainline
		 * driver hit pre-fix (SYS_INIT timeout, no IRQ).
		 */
		dev_dbg(core->dev, "Firmware status ack (FW_STATUS_RET)\n");
		complete(&core->fw_status_done);
		break;

	case VIDC_RESP_SYS_INIT:
		dev_info(core->dev, "Firmware initialized\n");
		complete(&core->sys_init_done);
		break;

	case VIDC_RESP_OPEN_CH:
		dev_dbg(core->dev, "Channel opened\n");
		if (inst) {
			inst->state = VIDC_STATE_OPEN;
			complete(&inst->done);
		}
		break;

	case VIDC_RESP_CLOSE_CH:
		dev_dbg(core->dev, "Channel closed\n");
		if (inst) {
			inst->state = VIDC_STATE_IDLE;
			complete(&inst->done);
		}
		break;

	case VIDC_RESP_SEQ_DONE:
		if (inst) {
			vidc_handle_seq_done(core, inst);
			/*
			 * SEQ_DONE post-processing (DPB alloc, INIT_BUFFERS,
			 * V4L2 event queue) can sleep — defer to a workqueue
			 * instead of completing the synchronous wait. The
			 * work runs in process context and ends with
			 * v4l2_m2m_job_finish so the m2m worker can pick up
			 * the next queued frame.
			 */
			queue_work(system_wq, &inst->seq_done_work);
		}
		break;

	case VIDC_RESP_FRAME_DONE:
		if (inst) {
			vidc_handle_frame_done(core, inst);
			inst->error = 0;
			/*
			 * FRAME_DONE → DPB copy → buf_done lives in
			 * frame_done_work because the memcpy and
			 * vb2_buf_done calls take vb2 queue locks that
			 * cannot be acquired from IRQ context.
			 */
			queue_work(system_wq, &inst->frame_done_work);
		}
		break;

	case VIDC_RESP_ENC_COMPLETE:
		if (inst) {
			vidc_handle_enc_complete(core, inst);
			inst->error = 0;
			/*
			 * Encoder-side analog of the decoder FRAME_DONE
			 * dispatch: post-frame work (buf_done, payload set,
			 * m2m_job_finish) lives in vidc_enc_complete_work
			 * because vb2 locks aren't acquirable from IRQ.
			 */
			queue_work(system_wq, &inst->enc_complete_work);
		}
		break;

	case VIDC_RESP_INIT_BUFFERS:
		dev_dbg(core->dev, "Buffers initialized\n");
		if (inst) {
			inst->state = VIDC_STATE_RUNNING;
			complete(&inst->done);
		}
		break;

	case VIDC_RESP_FLUSH_DONE:
		dev_dbg(core->dev, "Flush done\n");
		if (inst)
			complete(&inst->done);
		break;

	case VIDC_RESP_ERROR:
		dev_err(core->dev, "Firmware error: 0x%x\n", arg2);
		if (inst) {
			inst->error = -EIO;
			inst->state = VIDC_STATE_ERROR;
			complete(&inst->done);
		}
		break;

	default:
		break;
	}

	/*
	 * RISC2HOST_CMD already cleared at the top of the handler
	 * (before VIDC_REG_INTERRUPT) to match legacy ACK ordering.
	 * Don't re-clear here — leaves a window where a fresh response
	 * could be overwritten.
	 */

	spin_unlock_irqrestore(&core->irqlock, flags);

	return IRQ_HANDLED;
}

int vidc_load_firmware(struct vidc_core *core)
{
	int ret;

	dev_info(core->dev, "load_firmware: entry, fw_loaded=%d\n", core->fw_loaded);

	if (core->fw_loaded) {
		dev_info(core->dev, "load_firmware: already loaded, returning\n");
		return 0;
	}

	dev_info(core->dev, "load_firmware: requesting firmware %s\n", VIDC_FW_NAME);
	ret = request_firmware(&core->fw, VIDC_FW_NAME, core->dev);
	if (ret) {
		dev_err(core->dev, "failed to load firmware %s: %d\n",
			VIDC_FW_NAME, ret);
		return ret;
	}
	dev_info(core->dev, "load_firmware: got firmware, size=%zu\n", core->fw->size);

	if (core->fw->size > VIDC_FW_SIZE_MAX) {
		dev_err(core->dev, "firmware too large: %zu > %d\n",
			core->fw->size, VIDC_FW_SIZE_MAX);
		ret = -EINVAL;
		goto err_release_fw;
	}

	/*
	 * Place firmware + context pool + descriptor + shared-mem in SMI
	 * (System Memory Interface SRAM at 0x38000000). The RISC is designed
	 * to boot from SMI; it cannot execute from EBI/DRAM even though the
	 * interconnect topology permits it.
	 *
	 * CPU writes to SMI via ioremap DO reach physical SMI SRAM — confirmed
	 * by webOS DDL which uses the same ioremap+write approach (ddl_fw_init,
	 * vcd_ddl_utils.c). CPU reads from SMI return 0 (SMI is write-only
	 * from the CPU AXI path), so do not use readback to verify the write.
	 *
	 * 0x38000000 is already 128KB-aligned so fw_align_off is 0.
	 */
	core->ctxt_pool_size = VIDC_MAX_INSTANCES * VIDC_CTXT_MEM_SIZE;
	core->ctxt_pool_used = 0;
	core->fw_alloc_size = ALIGN(core->fw->size, SZ_128K)
			    + core->ctxt_pool_size
			    + VIDC_DESC_BUF_SIZE
			    + VIDC_SHM_SIZE;

	core->fw_dma_addr = ALIGN(core->fw_phys_base, SZ_128K);
	core->fw_align_off = core->fw_dma_addr - core->fw_phys_base;

	if (core->fw_dma_addr + core->fw_alloc_size >
	    core->fw_phys_base + core->fw_phys_size) {
		dev_err(core->dev,
			"fw+ctx (%zu bytes) exceeds SMI region (%zu bytes)\n",
			core->fw_alloc_size, core->fw_phys_size);
		ret = -ENOMEM;
		goto err_release_fw;
	}

	core->fw_vaddr = ioremap(core->fw_dma_addr, core->fw_alloc_size);
	if (!core->fw_vaddr) {
		ret = -ENOMEM;
		goto err_release_fw;
	}

	core->fw_size = core->fw->size;

	/*
	 * The VIDC embedded RISC is big-endian. The firmware blob shipped by
	 * the firmware-hp-tenderloin Yocto package (605 KB) is stored in
	 * big-endian byte order — the same byte order the RISC reads. Copy
	 * it verbatim with memcpy_toio; no byte-swapping needed.
	 *
	 * (The 500 KB webOS-doctor blob uses little-endian storage and would
	 * require swab32 per word, matching webOS ddl_fw_change_endian().
	 * The Yocto blob is the one on the target device.)
	 */
	memcpy_toio(core->fw_vaddr, core->fw->data, core->fw->size);

	/* CPU reads from SMI return 0; readback is not meaningful. */
	printk(KERN_EMERG "VIDC: SMI fw written: dma_addr=0x%08x alloc_size=%zu fw_size=%zu\n",
	       (u32)core->fw_dma_addr, core->fw_alloc_size, core->fw_size);

	/*
	 * Carve out per-channel scratch regions inside the firmware
	 * allocation. Each gets a fixed fw-relative offset across
	 * instance lifetimes:
	 *
	 *   layout: [fw (fw_size, 4K-aligned)]
	 *           [ctxt pool (VIDC_MAX_INSTANCES × 16 KB)]
	 *           [descriptor buffer (128 KB)]
	 *           [shared-memory region (4 KB)]
	 *
	 * Descriptor buffer hosts firmware scratch state during
	 * SEQ_HEADER parse and per-frame FRAME_DATA decode. Shared-memory
	 * region carries parameter blobs between host and firmware.
	 */
	core->desc_offset = ALIGN(core->fw_size, SZ_128K) + core->ctxt_pool_size;
	core->shm_offset = core->desc_offset + VIDC_DESC_BUF_SIZE;
	core->shm_vaddr = core->fw_vaddr + core->shm_offset;
	memset(core->fw_vaddr + core->desc_offset, 0, VIDC_DESC_BUF_SIZE);
	memset(core->shm_vaddr, 0, VIDC_SHM_SIZE);

	/*
	 * Explicitly clear the metadata-enable bitfield. Legacy DDL
	 * (vcd_ddl_metadata.c:356-393, ddl_vidc_metadata_enable) writes
	 * this on both SEQ_HEADER (vcd_ddl_vidc.c:200) and FRAME_DATA
	 * (line 573) paths.
	 *
	 * The memset above already wrote 0 to this offset, but legacy
	 * uses a *32-bit write* (DDL_MEM_WRITE_32), which on this
	 * non-coherent platform forces a store-buffer drain that
	 * memset+memcpy may not. Issue an explicit writel here so the
	 * firmware sees a definitive transaction on the metadata-enable
	 * cell before it reads any other SHM field.
	 *
	 * Bit layout:
	 *   bit 6: extradata pass-through
	 *   bit 5: encoder slice-size reporting
	 *   bit 4: VUI parameters
	 *   bit 3: SEI NAL data
	 *   bit 2: VC-1 parameters
	 *   bit 1: concealed-MB reporting
	 *   bit 0: per-MB QP array
	 *
	 * For plain "give me decoded NV12" all bits stay 0. None of
	 * the metadata streams are wired up to V4L2 extradata yet so
	 * enabling any of them would just waste firmware cycles.
	 */
	writel(0, core->shm_vaddr + VIDC_SHM_METADATA_ENABLE);

	core->fw_loaded = true;
	core->fw_running = false;

	/*
	 * Boot the on-chip RISC from the just-loaded DRAM buffer.
	 * Split out so vidc_runtime_resume() can re-issue it when the
	 * GDSC drop has wiped the firmware boot state.
	 */
	ret = vidc_boot_firmware(core);
	if (ret)
		goto err_free_dma;

	dev_info(core->dev,
		 "Firmware loaded at dma=0x%08x (%zu bytes), version 0x%08x\n",
		 (u32)core->fw_dma_addr, core->fw_size, core->fw_version);

	return 0;

err_free_dma:
	iounmap((void __iomem *)core->fw_vaddr);
	core->fw_vaddr = NULL;
	core->fw_loaded = false;
err_release_fw:
	release_firmware(core->fw);
	core->fw = NULL;
	return ret;
}

/*
 * Boot the on-chip RISC: program DRAM_BASE_A/B and send SYS_INIT.
 *
 * This is the part of firmware bring-up that must be repeated after
 * any GDSC drop (i.e., after every runtime suspend in the current
 * driver). The DRAM contents are preserved across suspend because
 * the firmware buffer lives in CMA-backed coherent memory, but the
 * RISC's own boot state is gone.
 *
 * Caller must ensure clocks are enabled. The function is idempotent
 * for fw_running=true: callers can pass through unconditionally
 * after a pm_runtime_resume_and_get() and the boot only happens
 * when needed.
 */
int vidc_boot_firmware(struct vidc_core *core)
{
	int ret;

	if (!core->fw_loaded || !core->fw_vaddr)
		return -EINVAL;

	if (core->fw_running)
		return 0;

	/*
	 * Fresh firmware boot: reset the context-pool cursor so the first
	 * OPEN_CH after a power cycle always gets slot 0. The DMA buffer
	 * is still mapped; slot contents are zeroed in vidc_open_channel
	 * before each use.
	 */
	core->ctxt_pool_used = 0;

	/*
	 * If the IRQ storm guard fired during a previous run it called
	 * disable_irq_nosync() and set irq_disabled_by_storm. Re-enable
	 * the IRQ now so the freshly-booted firmware can deliver responses.
	 * Reset the streak counter too so the guard can arm again if needed.
	 */
	if (core->irq_disabled_by_storm) {
		enable_irq(core->irq);
		core->irq_disabled_by_storm = false;
	}
	core->empty_irq_streak = 0;

	/*
	 * Bring the RISC out of reset and program firmware. We need
	 * vidc_hw_reset() here because the gdsc-qcom ved entry's
	 * LEGACY_FOOTSWITCH | SW_RESET only resets the AHB slave
	 * interface (VCODEC_AHB_RESET via mmcc) — NOT the internal
	 * RISC. Without our explicit SW_RESET dance ending in
	 * VIDC_RESET_NONE, the RISC stays in reset and SYS_INIT
	 * fires into the void.
	 *
	 * Empirical evidence: with hw_reset removed (commit
	 * 0e83a80e959c), VIDC_REG_FW_VERSION reads back as 0xdeadc0de
	 * (the placeholder value) and OPEN_CH later times out. With
	 * hw_reset back, the RISC actually executes the firmware and
	 * VIDC_REG_FW_VERSION reflects the real fw version.
	 *
	 * Both completions are armed before hw_reset so the IRQ
	 * dispatch can observe FW_STATUS_RET (fires inside hw_reset
	 * when the RISC is released) and SYS_INIT_RET. Both waits
	 * are lenient and non-fatal:
	 *  - FW_STATUS_RET: 200 ms (newer firmware blobs don't emit
	 *    this signal, so we proceed regardless)
	 *  - SYS_INIT_RET: 1 s (observation-only — downstream OPEN_CH
	 *    is the load-bearing health check)
	 */
	reinit_completion(&core->fw_status_done);
	reinit_completion(&core->sys_init_done);

	dev_info(core->dev, "boot_fw: about to call vidc_hw_reset (dram_base=0x%08x)\n",
		 (u32)core->fw_dma_addr);
	dev_info(core->dev, "boot_fw: pre-reset SW_RESET=0x%08x FW_VERSION=0x%08x\n",
		 vidc_read(core, VIDC_REG_SW_RESET),
		 vidc_read(core, VIDC_REG_FW_VERSION));
	ret = vidc_hw_reset(core, core->fw_dma_addr);
	if (ret) {
		dev_err(core->dev, "hw reset failed: %d\n", ret);
		return ret;
	}
	printk(KERN_EMERG "VIDC: boot_fw: about to read post-reset registers\n");
	dev_info(core->dev, "boot_fw: post-reset SW_RESET=0x%08x FW_VERSION=0x%08x\n",
		 vidc_read(core, VIDC_REG_SW_RESET),
		 vidc_read(core, VIDC_REG_FW_VERSION));
	printk(KERN_EMERG "VIDC: boot_fw: post-reset register reads completed\n");

	/*
	 * Poll RISC2HOST_CMD and INTERRUPT register directly to detect RISC
	 * activity independent of IRQ delivery. This distinguishes between:
	 *   a) RISC not executing (RISC2HOST_CMD stays 0, INTR stays 0)
	 *   b) RISC executing but IRQ not delivered to ARM
	 *      (RISC2HOST_CMD changes, INTR changes, but completion never fires)
	 */
	{
		int poll;
		u32 r2h, intr, fwver;

		for (poll = 0; poll < 200; poll++) {
			r2h   = vidc_read(core, VIDC_REG_RISC2HOST_CMD);
			intr  = vidc_read(core, VIDC_REG_INTERRUPT);
			fwver = vidc_read(core, VIDC_REG_FW_VERSION);
			if (r2h || intr || fwver) {
				printk(KERN_EMERG
				       "VIDC: poll[%d]: RISC2HOST_CMD=0x%08x INTR=0x%08x FW_VERSION=0x%08x\n",
				       poll, r2h, intr, fwver);
				break;
			}
			usleep_range(10000, 11000);
		}
		if (!r2h && !intr && !fwver)
			printk(KERN_EMERG "VIDC: poll: no RISC activity after 2s (RISC not executing)\n");
	}

	dev_info(core->dev, "boot_fw: hw_reset returned ok, waiting FW_STATUS_RET (2000ms)\n");

	{
		unsigned long ret_jif = wait_for_completion_timeout(
				&core->fw_status_done, msecs_to_jiffies(2000));
		dev_info(core->dev,
			 "boot_fw: FW_STATUS wait returned, jiffies_left=%lu (%s)\n",
			 ret_jif, ret_jif ? "got signal" : "timed out");
	}

	dev_info(core->dev, "boot_fw: about to send SYS_INIT (arg1=%zu)\n",
		 core->fw_alloc_size);
	ret = vidc_send_cmd(core, VIDC_CMD_SYS_INIT,
			    core->fw_alloc_size, 0, 0, 0);
	if (ret) {
		dev_err(core->dev, "failed to send SYS_INIT: %d\n", ret);
		return ret;
	}
	dev_info(core->dev, "boot_fw: SYS_INIT sent, waiting SYS_INIT_RET (1s)\n");

	{
		unsigned long ret_jif = wait_for_completion_timeout(
				&core->sys_init_done, msecs_to_jiffies(1000));
		dev_info(core->dev,
			 "boot_fw: SYS_INIT wait returned, jiffies_left=%lu (%s)\n",
			 ret_jif, ret_jif ? "got signal" : "timed out");
	}

	core->fw_running = true;
	core->fw_version = vidc_read(core, VIDC_REG_FW_VERSION);
	dev_info(core->dev, "boot_fw: done, FW_VERSION=0x%08x\n",
		 core->fw_version);
	return 0;
}

void vidc_unload_firmware(struct vidc_core *core)
{
	if (!core->fw_loaded)
		return;

	if (core->fw_vaddr) {
		iounmap((void __iomem *)core->fw_vaddr);
		core->fw_vaddr = NULL;
	}

	if (core->fw) {
		release_firmware(core->fw);
		core->fw = NULL;
	}

	core->fw_loaded = false;
}

/*
 * Map V4L2-side vidc_codec values to the RISC firmware's codec ID. The
 * mainline enum values are arranged to match the firmware's encoding
 * directly (decode 0..9, encode 16..18) so we can cast in most cases —
 * but make the mapping explicit so a future enum reorder doesn't
 * silently break the firmware handshake.
 */
static u32 vidc_codec_to_fw(enum vidc_codec codec)
{
	switch (codec) {
	case VIDC_CODEC_H264_DEC:	return 0;
	case VIDC_CODEC_VC1_DEC:	return 1;
	case VIDC_CODEC_MPEG4_DEC:	return 2;
	case VIDC_CODEC_MPEG2_DEC:	return 3;
	case VIDC_CODEC_H263_DEC:	return 4;
	case VIDC_CODEC_VC1_RCV_DEC:	return 5;
	case VIDC_CODEC_DIVX311_DEC:	return 6;
	case VIDC_CODEC_DIVX412_DEC:	return 7;
	case VIDC_CODEC_DIVX502_DEC:	return 8;
	case VIDC_CODEC_DIVX503_DEC:	return 9;
	case VIDC_CODEC_H264_ENC:	return 16;
	case VIDC_CODEC_MPEG4_ENC:	return 17;
	case VIDC_CODEC_H263_ENC:	return 18;
	default:			return 0;	/* default to H264 dec */
	}
}

/*
 * Open a channel with the on-chip RISC for one decoder/encoder instance.
 *
 * Allocates a 16 KB context buffer from the firmware-adjacent pool,
 * then issues HOST2RISC OPEN_CH with four arguments:
 *
 *   arg1: codec ID (see vidc_codec_to_fw)
 *   arg2: pixel-cache control (decode disables; encode enables)
 *   arg3: context-memory offset in DRAM_BASE_A, shifted right by 11
 *         (legacy DDL convention - matches CH0_Y_ADDR shift)
 *   arg4: context-memory size in bytes
 *
 * The on-chip RISC initialises its per-instance state in the context
 * buffer and acknowledges with RISC2HOST RESP_OPEN_CH, which the IRQ
 * handler turns into a completion on inst->done.
 *
 * Must be called with the channel currently closed (inst->ch_open == 0).
 */
int vidc_open_channel(struct vidc_inst *inst)
{
	struct vidc_core *core = inst->core;
	unsigned long flags;
	u32 fw_codec, pcache, ctxt_offset_shifted;
	int ret;

	if (inst->ch_open)
		return 0;

	/*
	 * Lazy firmware load. vidc_probe doesn't request_firmware (the
	 * V4L2 device-node registration shouldn't block on FS availability
	 * at boot), so the firmware blob isn't loaded and the per-instance
	 * context-memory pool isn't sized until the first user actually
	 * opens a channel. vidc_load_firmware is idempotent: it returns
	 * immediately if fw_loaded is already true. On first call here it
	 * does:
	 *   request_firmware(VIDC_FW_NAME)
	 *   dma_alloc_coherent for fw + ctxt pool + desc + shm
	 *   memcpy firmware to its 128 KB-aligned slot
	 *   sets ctxt_pool_size = VIDC_MAX_INSTANCES * VIDC_CTXT_MEM_SIZE
	 *   calls vidc_boot_firmware (which programs DRAM_BASE etc.)
	 * If we skipped this, the pool size stays 0 and the next check
	 * fails with "context-memory pool exhausted (0/0)".
	 *
	 * Called after the m2m start_streaming pm_runtime_resume_and_get,
	 * so GDSC + clocks are on and vidc_boot_firmware can safely write
	 * the boot-control registers.
	 */
	dev_info(core->dev, "open_ch: about to call vidc_load_firmware\n");
	ret = vidc_load_firmware(core);
	if (ret) {
		dev_err(core->dev, "open_ch: load_firmware failed: %d\n", ret);
		return ret;
	}
	dev_info(core->dev, "open_ch: load_firmware returned ok\n");

	mutex_lock(&core->lock);

	if (core->ctxt_pool_used + VIDC_CTXT_MEM_SIZE > core->ctxt_pool_size) {
		dev_err(core->dev, "context-memory pool exhausted (%zu/%zu)\n",
			core->ctxt_pool_used, core->ctxt_pool_size);
		ret = -ENOMEM;
		goto unlock;
	}

	/*
	 * Context buffers live in the firmware allocation, starting at
	 * ALIGN(fw_size, 128K) to clear the firmware's global data footprint.
	 * The Yocto blob's SYS_INIT_RET reports 0x97000 (618 KB) used;
	 * ALIGN(605 KB code, 128 KB) = 640 KB is safely past that.
	 */
	inst->ctxt_mem_offset = ALIGN(core->fw_size, SZ_128K)
			      + core->ctxt_pool_used;
	inst->ctxt_mem_vaddr = core->fw_vaddr + inst->ctxt_mem_offset;
	inst->ctxt_mem_dma_addr = core->fw_dma_addr + inst->ctxt_mem_offset;

	memset(inst->ctxt_mem_vaddr, 0, VIDC_CTXT_MEM_SIZE);

	core->ctxt_pool_used += VIDC_CTXT_MEM_SIZE;

	mutex_unlock(&core->lock);

	fw_codec = vidc_codec_to_fw(inst->codec);
	pcache = inst->decoder ? VIDC_PCACHE_DEC_DISABLE : VIDC_PCACHE_ENC_ENABLE;
	ctxt_offset_shifted = inst->ctxt_mem_offset >> VIDC_ADDR_SHIFT;

	spin_lock_irqsave(&core->irqlock, flags);
	core->curr_inst = inst;
	reinit_completion(&inst->done);
	inst->error = 0;
	spin_unlock_irqrestore(&core->irqlock, flags);

	dev_dbg(core->dev,
		"OPEN_CH codec=%u pcache=%u ctxt_off=0x%x sz=%u\n",
		fw_codec, pcache, ctxt_offset_shifted, VIDC_CTXT_MEM_SIZE);

	dev_info(core->dev,
		 "open_ch: sending OPEN_CH codec=%u pcache=%u ctxt_off=0x%x\n",
		 fw_codec, pcache, ctxt_offset_shifted);
	ret = vidc_send_cmd(core, VIDC_CMD_OPEN_CH, fw_codec, pcache,
			    ctxt_offset_shifted, VIDC_CTXT_MEM_SIZE);
	if (ret) {
		dev_err(core->dev, "OPEN_CH send failed: %d\n", ret);
		goto release_ctxt;
	}
	dev_info(core->dev, "open_ch: OPEN_CH sent, waiting RESP_OPEN_CH (1s)\n");

	{
		unsigned long ret_jif = wait_for_completion_timeout(
				&inst->done, msecs_to_jiffies(1000));
		dev_info(core->dev,
			 "open_ch: wait returned, jiffies_left=%lu (%s)\n",
			 ret_jif, ret_jif ? "got signal" : "timed out");
		if (!ret_jif) {
			dev_err(core->dev, "OPEN_CH timeout\n");
			ret = -ETIMEDOUT;
			goto release_ctxt;
		}
	}

	if (inst->error) {
		dev_err(core->dev, "OPEN_CH firmware error: %d\n",
			inst->error);
		ret = inst->error;
		goto release_ctxt;
	}

	inst->ch_open = true;
	dev_info(core->dev,
		 "VIDC channel opened (codec=%u, ctxt off=0x%x sz=%u)\n",
		 fw_codec, inst->ctxt_mem_offset, VIDC_CTXT_MEM_SIZE);
	return 0;

release_ctxt:
	mutex_lock(&core->lock);
	core->ctxt_pool_used -= VIDC_CTXT_MEM_SIZE;
unlock:
	mutex_unlock(&core->lock);
	return ret;
}

/*
 * Close the channel previously opened with vidc_open_channel().
 *
 * Sends HOST2RISC CLOSE_CH with no codec arg and waits for the
 * RESP_CLOSE_CH acknowledgement. Recycles the context pool cursor so
 * sequential single-instance use reuses slot 0.
 *
 * Safe to call when the channel is already closed (returns 0).
 */
int vidc_close_channel(struct vidc_inst *inst)
{
	struct vidc_core *core = inst->core;
	unsigned long flags;
	int ret;

	if (!inst->ch_open)
		return 0;

	/*
	 * Tear down DPB before closing the firmware channel. The CLOSE_CH
	 * command releases per-instance state on the RISC; if we did it
	 * before freeing the DPB, the firmware might still hold dangling
	 * references into our about-to-be-freed pool.
	 */
	vidc_free_buffers(inst);

	spin_lock_irqsave(&core->irqlock, flags);
	core->curr_inst = inst;
	reinit_completion(&inst->done);
	inst->error = 0;
	spin_unlock_irqrestore(&core->irqlock, flags);

	ret = vidc_send_cmd(core, VIDC_CMD_CLOSE_CH, 0, 0, 0, 0);
	if (ret) {
		dev_err(core->dev, "CLOSE_CH send failed: %d\n", ret);
		return ret;
	}

	if (!wait_for_completion_timeout(&inst->done,
					 msecs_to_jiffies(1000))) {
		dev_err(core->dev, "CLOSE_CH timeout\n");
		return -ETIMEDOUT;
	}

	inst->ch_open = false;

	/*
	 * Recycle the context pool slot. This makes sequential single-instance
	 * use reuse slot 0 on every open/close cycle instead of walking the
	 * pool forward until it exhausts. The context DMA memory is still
	 * mapped; it will be zeroed again in vidc_open_channel before the
	 * next OPEN_CH.
	 *
	 * Concurrent multi-instance callers must close in LIFO order for
	 * this to remain correct (last-opened instance is closed first),
	 * which matches the normal V4L2 usage pattern. VIDC_MAX_INSTANCES
	 * caps concurrent opens to 2 so the worst-case wasted stride is
	 * one VIDC_CTXT_MEM_SIZE slot.
	 */
	mutex_lock(&core->lock);
	if (core->ctxt_pool_used >= VIDC_CTXT_MEM_SIZE)
		core->ctxt_pool_used -= VIDC_CTXT_MEM_SIZE;
	mutex_unlock(&core->lock);

	/*
	 * Reset the sequence-parsed gate so a subsequent STREAMON cycle
	 * re-parses whatever bitstream is fed in next. Real userspace
	 * (e.g. gstreamer playlists) closes + reopens between segments
	 * and the new segment may have completely different SPS.
	 */
	inst->seq_parsed = false;
	inst->state = VIDC_STATE_IDLE;
	inst->seq_width = 0;
	inst->seq_height = 0;
	inst->min_dpb_count = 0;

	dev_dbg(core->dev, "VIDC channel closed\n");
	return inst->error;
}

/*
 * Compute tile-NV12 plane sizes for one DPB slot. Width is rounded up
 * to 128 px, height to 32 px; chroma plane is half the luma plane.
 *
 * Matches webos-linux-kernel-touchpad/drivers/video/msm/vidc/1080p/ddl/
 * vcd_ddl_helper.c::ddl_get_yuv_buf_size for the tile path. We do NOT
 * apply DDL_TILE_MULTIPLY_FACTOR alignment on top — the legacy DDL did
 * that to handle pixel-cache granularity which is hardware-internal
 * and irrelevant to the buffer allocator.
 */
static void vidc_dpb_calc_sizes(u32 width, u32 height,
				u32 *y_size, u32 *c_size)
{
	u32 w = ALIGN(width, VIDC_DPB_TILE_ALIGN_WIDTH);
	u32 h = ALIGN(height, VIDC_DPB_TILE_ALIGN_HEIGHT);

	*y_size = w * h;
	*c_size = (w * h) / 2;
}

/*
 * Allocate and program the DPB (display picture buffer) pool, then
 * issue the VIDC_OP_INIT_BUFFERS command to the firmware.
 *
 * Pre-conditions:
 *   - vidc_open_channel() has acked (inst->ch_open == true)
 *   - vidc_handle_seq_done() ran and populated inst->seq_width /
 *     seq_height / min_dpb_count (i.e. inst->seq_parsed is true)
 *
 * Steps:
 *   1. Compute per-slot Y / C / MV sizes from seq dimensions
 *   2. Allocate one contiguous DMA block holding dpb_count copies of
 *      (Y + C + MV) so we can program slot offsets as fw-relative
 *      and the firmware can stride through the pool with one base
 *   3. For each slot, write Y / C / MV register-field values
 *      (offset_from_fw_dma_addr >> VIDC_ADDR_SHIFT) into the
 *      DPB_*_BASE register arrays
 *   4. Send HOST2RISC INIT_BUFFERS via CH0_INST_ID and wait
 *      on RESP_INIT_BUFFERS
 *
 * On any failure the entire allocation is unwound — partial state
 * would leave the firmware confused about how many DPB slots exist.
 */
int vidc_init_buffers(struct vidc_inst *inst)
{
	struct vidc_core *core = inst->core;
	unsigned long flags;
	u32 i;
	u32 y_size, c_size, mv_size, slot_size, total_size;
	dma_addr_t slot_base;
	u32 fw_relative;
	int ret;

	if (inst->dpb_inited)
		return 0;

	if (!inst->seq_parsed) {
		dev_err(core->dev,
			"vidc_init_buffers called before SEQ parse\n");
		return -EINVAL;
	}

	if (!inst->seq_width || !inst->seq_height) {
		dev_err(core->dev,
			"vidc_init_buffers: bitstream geometry not set\n");
		return -EINVAL;
	}

	inst->dpb_count = inst->min_dpb_count;
	if (!inst->dpb_count)
		inst->dpb_count = 4;	/* sane default if firmware skipped it */
	if (inst->dpb_count > VIDC_DPB_REG_SLOTS)
		inst->dpb_count = VIDC_DPB_REG_SLOTS;

	vidc_dpb_calc_sizes(inst->seq_width, inst->seq_height,
			    &y_size, &c_size);
	mv_size = (inst->codec == VIDC_CODEC_H264_DEC) ?
		  VIDC_DPB_MV_SIZE : 0;

	inst->dpb_y_size = y_size;
	inst->dpb_c_size = c_size;
	inst->dpb_mv_size = mv_size;

	slot_size = ALIGN(y_size + c_size + mv_size, SZ_4K);
	total_size = slot_size * inst->dpb_count;

	inst->dpb_y_vaddr = dma_alloc_coherent(core->dev, total_size,
					       &inst->dpb_y_dma_addr,
					       GFP_KERNEL);
	if (!inst->dpb_y_vaddr) {
		dev_err(core->dev,
			"DPB pool alloc failed (%u slots × %u bytes)\n",
			inst->dpb_count, slot_size);
		return -ENOMEM;
	}
	inst->dpb_y_alloc_size = total_size;

	/*
	 * DPB pool must be addressable as a positive offset from
	 * fw_dma_addr — the register field is unsigned. CMA usually hands
	 * out high addresses, but verify explicitly.
	 */
	if (inst->dpb_y_dma_addr < core->fw_dma_addr) {
		dev_err(core->dev,
			"DPB pool below firmware base (%pad < %pad)\n",
			&inst->dpb_y_dma_addr, &core->fw_dma_addr);
		ret = -ERANGE;
		goto err_free_dma;
	}

	/* Program DPB register slots */
	for (i = 0; i < inst->dpb_count; i++) {
		slot_base = inst->dpb_y_dma_addr + i * slot_size;
		fw_relative = slot_base - core->fw_dma_addr;

		vidc_write(core, VIDC_REG_DPB_LUMA_BASE + i * 4,
			   fw_relative >> VIDC_ADDR_SHIFT);
		vidc_write(core, VIDC_REG_DPB_CHROMA_BASE + i * 4,
			   (fw_relative + y_size) >> VIDC_ADDR_SHIFT);
		if (mv_size)
			vidc_write(core, VIDC_REG_DPB_MV_BASE + i * 4,
				   (fw_relative + y_size + c_size)
				    >> VIDC_ADDR_SHIFT);
	}

	dev_info(core->dev,
		 "DPB pool: %u slots × (y=%u c=%u mv=%u), total %u bytes at %pad\n",
		 inst->dpb_count, y_size, c_size, mv_size, total_size,
		 &inst->dpb_y_dma_addr);

	/*
	 * Publish the per-slot buffer sizes via the shared-memory region.
	 * The firmware reads these on INIT_BUFFERS to compute its own
	 * per-slot strides; without them the legacy DDL trace shows the
	 * firmware ack'ing INIT_BUFFERS with an "alloc size mismatch" error
	 * (vcd_ddl_errors.c). Offsets are part of the firmware ABI -
	 * mirror the legacy VIDC_SM_ALLOCATED_*_DPB_SIZE_ADDR constants.
	 */
	writel(y_size,
	       core->shm_vaddr + VIDC_SHM_ALLOCATED_LUMA_DPB_SIZE);
	writel(c_size,
	       core->shm_vaddr + VIDC_SHM_ALLOCATED_CHROMA_DPB_SIZE);
	if (mv_size)
		writel(mv_size,
		       core->shm_vaddr + VIDC_SHM_ALLOCATED_MV_SIZE);

	/* Issue INIT_BUFFERS command */
	spin_lock_irqsave(&core->irqlock, flags);
	core->curr_inst = inst;
	reinit_completion(&inst->done);
	inst->error = 0;
	spin_unlock_irqrestore(&core->irqlock, flags);

	/*
	 * Point the firmware at the shared-memory region we just populated.
	 * Value is byte offset from fw_dma_addr (no shift). Every command
	 * that exchanges parameters via SHM needs this; for now we only
	 * issue it before commands that read SHM (INIT_BUFFERS here, and
	 * SEQ_HEADER / FRAME_DATA in vidc_dec_submit_frame). Once async
	 * device_run lands the SHM register should be written from a
	 * common helper.
	 */
	vidc_write(core, VIDC_REG_CH0_SHARED_MEM, core->shm_offset);

	/* Sequence number + DPB count visible to the firmware */
	core->cmd_seq_num++;
	vidc_write(core, VIDC_REG_CH0_CMD_SEQ_NUM, core->cmd_seq_num);
	vidc_write(core, VIDC_REG_CH0_DPB_CONFIG, inst->dpb_count);

	/* Kick INIT_BUFFERS via the operation-type bits in INST_ID */
	vidc_write(core, VIDC_REG_CH0_INST_ID,
		   VIDC_OP_INIT_BUFFERS | inst->inst_id);

	if (!wait_for_completion_timeout(&inst->done,
					 msecs_to_jiffies(1000))) {
		dev_err(core->dev, "INIT_BUFFERS timeout\n");
		ret = -ETIMEDOUT;
		goto err_free_dma;
	}

	if (inst->error) {
		dev_err(core->dev, "INIT_BUFFERS firmware error: %d\n",
			inst->error);
		ret = inst->error;
		goto err_free_dma;
	}

	inst->dpb_inited = true;
	dev_info(core->dev, "VIDC DPB initialised, %u slots active\n",
		 inst->dpb_count);
	return 0;

err_free_dma:
	dma_free_coherent(core->dev, inst->dpb_y_alloc_size,
			  inst->dpb_y_vaddr, inst->dpb_y_dma_addr);
	inst->dpb_y_vaddr = NULL;
	inst->dpb_y_dma_addr = 0;
	inst->dpb_y_alloc_size = 0;
	return ret;
}

/*
 * Copy a displayed DPB slot to the userspace CAPTURE buffer.
 *
 * After a successful FRAME_DONE, the firmware has filled one of our
 * internal DPB slots with the decoded frame (tile-NV12 layout). The
 * IRQ handler captured the slot's fw-relative offset into
 * inst->display_y_raw (luma) and inst->display_c_raw (chroma), both
 * encoded as offset_from_fw_dma_addr >> VIDC_ADDR_SHIFT.
 *
 * Reverse that encoding to find which DPB slot vaddr to read from,
 * then memcpy Y then C into the dst buffer. The data we copy is in
 * tile-NV12 layout — userspace consumers expecting linear NV12 need
 * to detile (or we expose V4L2_PIX_FMT_NV12MT, a follow-up).
 *
 * out_payload receives the byte count actually written (y_size + c_size).
 */
int vidc_copy_dpb_to_dst(struct vidc_inst *inst, void *dst_vaddr,
			 size_t dst_size, size_t *out_payload)
{
	struct vidc_core *core = inst->core;
	u32 y_offset, c_offset, slot_size, slot_idx;
	size_t y_size, c_size, frame_size;
	void *slot_y, *slot_c;
	dma_addr_t slot_phys;

	if (!inst->dpb_inited || !inst->dpb_y_vaddr) {
		dev_err(core->dev, "copy_dpb_to_dst: DPB not initialised\n");
		return -EINVAL;
	}

	y_offset = inst->display_y_raw << VIDC_ADDR_SHIFT;
	c_offset = inst->display_c_raw << VIDC_ADDR_SHIFT;
	y_size = inst->dpb_y_size;
	c_size = inst->dpb_c_size;
	frame_size = y_size + c_size;

	if (dst_size < frame_size) {
		dev_err(core->dev,
			"dst buffer too small: %zu < %zu\n",
			dst_size, frame_size);
		return -ENOSPC;
	}

	/*
	 * Translate fw-relative luma offset back to a DPB slot index.
	 * slot_size matches the per-slot stride from vidc_init_buffers():
	 *   ALIGN(y_size + c_size + mv_size, SZ_4K)
	 * We re-derive it from dpb_y_alloc_size / dpb_count rather than
	 * re-aligning so any future allocator change stays consistent.
	 */
	slot_size = inst->dpb_y_alloc_size / inst->dpb_count;
	slot_phys = core->fw_dma_addr + y_offset;

	if (slot_phys < inst->dpb_y_dma_addr ||
	    slot_phys >= inst->dpb_y_dma_addr + inst->dpb_y_alloc_size) {
		dev_err(core->dev,
			"display Y phys %pad outside DPB pool [%pad..+%zu]\n",
			&slot_phys, &inst->dpb_y_dma_addr,
			inst->dpb_y_alloc_size);
		return -EFAULT;
	}

	slot_idx = (slot_phys - inst->dpb_y_dma_addr) / slot_size;
	if (slot_idx >= inst->dpb_count) {
		dev_err(core->dev, "computed slot %u >= count %u\n",
			slot_idx, inst->dpb_count);
		return -EFAULT;
	}

	slot_y = inst->dpb_y_vaddr + slot_idx * slot_size;
	slot_c = slot_y + y_size;

	/*
	 * Sanity-check the chroma offset matches the slot we picked.
	 * If the firmware reported a chroma plane from a different slot
	 * than the luma plane, something is very wrong — bail rather
	 * than copy mismatched halves.
	 */
	if (c_offset != y_offset + y_size) {
		dev_warn(core->dev,
			 "luma/chroma offset mismatch: y=0x%x c=0x%x (expected c=0x%x)\n",
			 y_offset, c_offset, y_offset + (u32)y_size);
	}

	memcpy(dst_vaddr, slot_y, y_size);
	memcpy(dst_vaddr + y_size, slot_c, c_size);

	if (out_payload)
		*out_payload = frame_size;

	dev_dbg(core->dev,
		"copy_dpb_to_dst: slot=%u y=%zu c=%zu total=%zu\n",
		slot_idx, y_size, c_size, frame_size);

	return 0;
}

/*
 * Issue VIDC_CMD_FLUSH to discard in-flight buffers without tearing
 * down the channel. Used by V4L2_DEC_CMD_STOP (drain) and as a
 * recovery primitive after an error response.
 *
 * Caller must hold no irq-side state - this function blocks on the
 * RESP_FLUSH_DONE completion (the IRQ handler already handles
 * RESP_FLUSH_DONE → complete(inst->done)).
 *
 * flush_type: VIDC_FLUSH_INPUT, VIDC_FLUSH_OUTPUT, or VIDC_FLUSH_ALL.
 *   Legacy doesn't expose named constants for these; the bitmap is
 *   inferred from convention. If the firmware rejects the value
 *   it will surface as a RESP_ERROR in the IRQ.
 */
int vidc_flush_channel(struct vidc_inst *inst, u32 flush_type)
{
	struct vidc_core *core = inst->core;
	unsigned long flags;
	int ret;

	if (!inst->ch_open) {
		dev_warn(core->dev, "flush_channel on closed channel\n");
		return -EINVAL;
	}

	/* Flush type goes into the per-flush SHM cell; INBUF1/2 are
	 * for partial-input flushes (specific input buffers), unused
	 * here for the full-flush case. */
	writel(flush_type, core->shm_vaddr + VIDC_SHM_FLUSH_CMD_TYPE);
	writel(0, core->shm_vaddr + VIDC_SHM_FLUSH_CMD_INBUF1);
	writel(0, core->shm_vaddr + VIDC_SHM_FLUSH_CMD_INBUF2);

	spin_lock_irqsave(&core->irqlock, flags);
	core->curr_inst = inst;
	reinit_completion(&inst->done);
	inst->error = 0;
	spin_unlock_irqrestore(&core->irqlock, flags);

	vidc_write(core, VIDC_REG_CH0_SHARED_MEM, core->shm_offset);
	core->cmd_seq_num++;
	vidc_write(core, VIDC_REG_CH0_CMD_SEQ_NUM, core->cmd_seq_num);

	ret = vidc_send_cmd(core, VIDC_CMD_FLUSH, flush_type, 0, 0, 0);
	if (ret) {
		dev_err(core->dev, "FLUSH send failed: %d\n", ret);
		return ret;
	}

	if (!wait_for_completion_timeout(&inst->done,
					 msecs_to_jiffies(1000))) {
		dev_err(core->dev, "FLUSH timeout\n");
		return -ETIMEDOUT;
	}

	if (inst->error) {
		dev_err(core->dev, "FLUSH firmware error: %d\n", inst->error);
		return inst->error;
	}

	dev_dbg(core->dev, "VIDC channel flushed (type=0x%x)\n", flush_type);
	return 0;
}

/*
 * Send encoder SEQ_HEADER command and wait for SEQ_DONE.
 *
 * The webOS DDL encoder state machine requires a SEQ_HEADER round-trip
 * between OPEN_CH and INIT_BUFFERS. Without it, the firmware's rate-
 * control and codec-state initialisation haven't run, and INIT_BUFFERS
 * fails with error 0x51 (DIVIDE_BY_ZERO).
 *
 * Allocates a small DMA buffer to receive the generated SPS/PPS; the
 * caller can ignore its contents — we just need the SEQ_DONE ack.
 */
int vidc_enc_send_seq_header(struct vidc_inst *inst)
{
	struct vidc_core *core = inst->core;
	void *hdr_vaddr;
	dma_addr_t hdr_dma;
	unsigned long flags;
	int ret;

	hdr_vaddr = dma_alloc_coherent(core->dev, SZ_4K, &hdr_dma, GFP_KERNEL);
	if (!hdr_vaddr)
		return -ENOMEM;

	spin_lock_irqsave(&core->irqlock, flags);
	core->curr_inst = inst;
	reinit_completion(&inst->done);
	inst->error = 0;
	vidc_write(core, VIDC_REG_RISC2HOST_CMD, VIDC_RESP_EMPTY);
	vidc_write(core, VIDC_REG_CH0_STREAM_ADDR, hdr_dma >> VIDC_ADDR_SHIFT);
	vidc_write(core, VIDC_REG_CH0_STREAM_BUF_SIZE, SZ_4K);
	vidc_write(core, VIDC_REG_CH0_DESC_ADDR,
		   core->desc_offset >> VIDC_ADDR_SHIFT);
	vidc_write(core, VIDC_REG_CH0_DESC_BUF_SIZE, VIDC_DESC_BUF_SIZE);
	vidc_write(core, VIDC_REG_CH0_SHARED_MEM, core->shm_offset);
	core->cmd_seq_num++;
	vidc_write(core, VIDC_REG_CH0_CMD_SEQ_NUM, core->cmd_seq_num);
	vidc_write(core, VIDC_REG_CH0_INST_ID,
		   VIDC_OP_SEQ_HEADER | inst->inst_id);
	spin_unlock_irqrestore(&core->irqlock, flags);

	if (!wait_for_completion_timeout(&inst->done, msecs_to_jiffies(1000))) {
		dev_err(core->dev, "encoder SEQ_HEADER timeout\n");
		ret = -ETIMEDOUT;
		goto err_free;
	}

	if (inst->error) {
		dev_err(core->dev, "encoder SEQ_HEADER error: %d\n",
			inst->error);
		ret = inst->error;
		goto err_free;
	}

	dev_info(core->dev, "encoder SEQ_HEADER done\n");
	ret = 0;

err_free:
	dma_free_coherent(core->dev, SZ_4K, hdr_vaddr, hdr_dma);
	return ret;
}

/*
 * Encoder analog of vidc_init_buffers.
 *
 * Allocates recon (reconstruction) buffers - the encoder's analog
 * to the decoder's DPB. The encoder reads from the source frame
 * provided via the OUTPUT queue and writes its reconstructed
 * reference frames into these slots so subsequent inter-predicted
 * frames have something to look back at.
 *
 * Pre-conditions:
 *   - vidc_open_channel() has acked (inst->ch_open == true)
 *   - inst->out_width / out_height set via VIDIOC_S_FMT
 *   - vidc_enc_send_seq_header() has been called and returned success
 *
 * Hardware register layout differs from decoder DPB:
 *   - DPB_LUMA / CHROMA / MV are 3 separate register arrays at
 *     0x300 / 0x380 / 0x400 with 4-byte stride per slot
 *   - RECON_LUMA / CHROMA are interleaved at 0x480 with 8-byte
 *     stride per slot (LUMA_i at 0x480 + i*8, CHROMA_i at +0x484 + i*8)
 *
 * Slot count is fixed at 4 (VIDC_MAX_RECON_BUFFERS) - matches
 * legacy vcd_ddl_vidc.c:473 `const u32 recon_bufs = 4;`. No firmware
 * SEQ_DONE feedback like the decoder; for H.264 baseline 2 recon
 * suffice but 4 covers Main/High profiles with B-frames.
 *
 * Re-uses the dpb_* fields in struct vidc_inst for tracking - a
 * single instance is either decoder OR encoder, never both.
 */
int vidc_init_enc_buffers(struct vidc_inst *inst)
{
	struct vidc_core *core = inst->core;
	unsigned long flags;
	u32 i;
	u32 y_size, c_size, slot_size, total_size;
	dma_addr_t slot_base;
	u32 fw_relative;
	int ret;

	if (inst->dpb_inited)
		return 0;

	if (!inst->out_width || !inst->out_height) {
		dev_err(core->dev,
			"vidc_init_enc_buffers: encode geometry not set\n");
		return -EINVAL;
	}

	inst->dpb_count = VIDC_MAX_RECON_BUFFERS;

	vidc_dpb_calc_sizes(inst->out_width, inst->out_height,
			    &y_size, &c_size);

	inst->dpb_y_size = y_size;
	inst->dpb_c_size = c_size;
	inst->dpb_mv_size = 0;	/* MV/col-zero buffers not yet wired */

	slot_size = ALIGN(y_size + c_size, SZ_4K);
	total_size = slot_size * inst->dpb_count;

	inst->dpb_y_vaddr = dma_alloc_coherent(core->dev, total_size,
					       &inst->dpb_y_dma_addr,
					       GFP_KERNEL);
	if (!inst->dpb_y_vaddr) {
		dev_err(core->dev,
			"recon pool alloc failed (%u slots × %u bytes)\n",
			inst->dpb_count, slot_size);
		return -ENOMEM;
	}
	inst->dpb_y_alloc_size = total_size;

	if (inst->dpb_y_dma_addr < core->fw_dma_addr) {
		dev_err(core->dev,
			"recon pool below firmware base (%pad < %pad)\n",
			&inst->dpb_y_dma_addr, &core->fw_dma_addr);
		ret = -ERANGE;
		goto err_free_dma;
	}

	/*
	 * Program RECON register slots. Layout differs from DPB: each
	 * slot is a (LUMA, CHROMA) pair at 8-byte stride starting from
	 * VIDC_REG_RECON_LUMA_0 = 0x480.
	 */
	for (i = 0; i < inst->dpb_count; i++) {
		slot_base = inst->dpb_y_dma_addr + i * slot_size;
		fw_relative = slot_base - core->fw_dma_addr;

		vidc_write(core, VIDC_REG_RECON_LUMA_0 + i * 8,
			   fw_relative >> VIDC_ADDR_SHIFT);
		vidc_write(core, VIDC_REG_RECON_CHROMA_0 + i * 8,
			   (fw_relative + y_size) >> VIDC_ADDR_SHIFT);
	}

	dev_info(core->dev,
		 "recon pool: %u slots × (y=%u c=%u), total %u bytes at %pad\n",
		 inst->dpb_count, y_size, c_size, total_size,
		 &inst->dpb_y_dma_addr);

	/* Publish sizes to SHM (same offsets as decoder uses) */
	writel(y_size,
	       core->shm_vaddr + VIDC_SHM_ALLOCATED_LUMA_DPB_SIZE);
	writel(c_size,
	       core->shm_vaddr + VIDC_SHM_ALLOCATED_CHROMA_DPB_SIZE);

	spin_lock_irqsave(&core->irqlock, flags);
	core->curr_inst = inst;
	reinit_completion(&inst->done);
	inst->error = 0;
	spin_unlock_irqrestore(&core->irqlock, flags);

	vidc_write(core, VIDC_REG_CH0_SHARED_MEM, core->shm_offset);
	core->cmd_seq_num++;
	vidc_write(core, VIDC_REG_CH0_CMD_SEQ_NUM, core->cmd_seq_num);
	vidc_write(core, VIDC_REG_CH0_DPB_CONFIG, inst->dpb_count);

	vidc_write(core, VIDC_REG_CH0_INST_ID,
		   VIDC_OP_INIT_BUFFERS | inst->inst_id);

	if (!wait_for_completion_timeout(&inst->done,
					 msecs_to_jiffies(1000))) {
		dev_err(core->dev, "encoder INIT_BUFFERS timeout\n");
		ret = -ETIMEDOUT;
		goto err_free_dma;
	}

	if (inst->error) {
		dev_err(core->dev,
			"encoder INIT_BUFFERS firmware error: %d\n",
			inst->error);
		ret = inst->error;
		goto err_free_dma;
	}

	inst->dpb_inited = true;
	dev_info(core->dev,
		 "VIDC encoder recon initialised, %u slots active\n",
		 inst->dpb_count);
	return 0;

err_free_dma:
	dma_free_coherent(core->dev, inst->dpb_y_alloc_size,
			  inst->dpb_y_vaddr, inst->dpb_y_dma_addr);
	inst->dpb_y_vaddr = NULL;
	inst->dpb_y_dma_addr = 0;
	inst->dpb_y_alloc_size = 0;
	return ret;
}

/*
 * Apply codec-specific configuration that the firmware can't auto-
 * derive from the bitstream. Called after vidc_open_channel() and
 * before the first SEQ_HEADER submission so any per-codec register
 * writes happen on a freshly-opened channel.
 *
 * Codec-specific knobs from legacy DDL (vcd_ddl_vidc.c +
 * vcd_ddl_properties.c):
 *
 *   H.264         — none (firmware parses entropy_sel / profile /
 *                   level from the SPS itself)
 *   MPEG-4 / H.263— post-loop-filter control (legacy
 *                   vidc_1080p_set_decode_mpeg4_pp_filter; 2-bit
 *                   LF_CONTROL field at legacy REG_152500 / +0x848)
 *   DivX 3        — manual width/height override (legacy
 *                   vidc_1080p_set_decode_divx3_resolution_ch0;
 *                   width at legacy REG_175608, height at REG_612810
 *                   / +0x2050); the bitstream lacks resolution info
 *                   so the host must supply it
 *   VC-1          — RCV-format resolution swap
 *   MPEG-2        — none
 *
 * Currently only the H.264 path is exercised end-to-end. The
 * non-H.264 entries are wired as stubs with WARN_ONCE so a user
 * trying to feed those codecs into the driver gets a clear hint
 * that the codec-specific config is incomplete, rather than a
 * silent FRAME_DATA stall. Each stub also documents the legacy
 * register offset that needs decoding against this kernel's
 * vidc_core.h to be enabled.
 */
/*
 * Apply per-encoder configuration that the firmware needs at session
 * open time. The mainline submit_frame already writes per-frame
 * mutable settings (width/height/bitrate/framerate); this function
 * writes the session-stable settings (profile/level, rate-control
 * config, reaction coefficient, QP range).
 *
 * Values are conservative defaults until v4l2 controls land:
 *
 *   PROFILE_LEVEL  : codec-dependent encoding. H.264 packs profile
 *                    in the high byte (Baseline=1) and level in the
 *                    low byte (3.0=30). MPEG-4 / H.263 use their own
 *                    profile encodings.
 *   RC_CONFIG      : 0 = CBR (constant bitrate). VBR / off / disabled
 *                    are non-zero values in the legacy enum but
 *                    documented values aren't easily mapped without
 *                    a datasheet.
 *   REACTION_COEFF : 0x14 (= 20). Higher = slower rate-control
 *                    response. Conservative default; legacy chose
 *                    this value for streaming-oriented profiles.
 *   QP_RANGE       : packs (max_qp << 16) | min_qp. H.264 range is
 *                    0..51; using 10..40 keeps quality acceptable
 *                    without driving bitrate to absurd levels.
 *
 * Returns 0 even for unsupported codecs (firmware uses its defaults).
 */
int vidc_apply_enc_codec_config(struct vidc_inst *inst)
{
	struct vidc_core *core = inst->core;
	u32 profile_level;
	u32 qp_range;

	switch (inst->codec) {
	case VIDC_CODEC_H264_ENC:
		/* Baseline profile, Level 3.0 (1080p capable: L4.0=40) */
		profile_level = (1 << 8) | 30;
		break;

	case VIDC_CODEC_MPEG4_ENC:
		/* Simple Profile, Level 5 (common for SD video) */
		profile_level = (0 << 8) | 5;
		break;

	case VIDC_CODEC_H263_ENC:
		/* Baseline profile, Level 70 (legacy default) */
		profile_level = (0 << 8) | 70;
		break;

	default:
		dev_warn_once(core->dev,
			      "unknown encoder codec %d - profile/level left default\n",
			      inst->codec);
		profile_level = 0;
		break;
	}

	qp_range = (40 << 16) | 10;	/* min=10, max=40 */

	vidc_write(core, VIDC_REG_ENC_FRAME_WIDTH, inst->out_width);
	vidc_write(core, VIDC_REG_ENC_FRAME_HEIGHT, inst->out_height);
	vidc_write(core, VIDC_REG_ENC_TARGET_BITRATE, inst->bitrate);
	/* Firmware expects framerate in millifps (fps * 1000) */
	vidc_write(core, VIDC_REG_ENC_FRAME_RATE, inst->framerate * 1000);
	vidc_write(core, VIDC_REG_ENC_PROFILE_LEVEL, profile_level);
	vidc_write(core, VIDC_REG_ENC_RC_CONFIG, 0);		/* CBR */
	vidc_write(core, VIDC_REG_ENC_REACTION_COEFF, 0x14);
	vidc_write(core, VIDC_REG_ENC_QP_RANGE, qp_range);

	dev_dbg(core->dev,
		"encoder config: %ux%u fps=%u bitrate=%u profile_level=0x%x\n",
		inst->out_width, inst->out_height,
		inst->framerate, inst->bitrate, profile_level);

	return 0;
}

int vidc_apply_dec_codec_config(struct vidc_inst *inst)
{
	struct vidc_core *core = inst->core;

	switch (inst->codec) {
	case VIDC_CODEC_H264_DEC:
		/* Firmware reads entropy/profile/level from SPS itself */
		return 0;

	case VIDC_CODEC_MPEG2_DEC:
		/* No host-side codec config required */
		return 0;

	case VIDC_CODEC_MPEG4_DEC:
	case VIDC_CODEC_H263_DEC:
		/*
		 * Enable the post-loop deblock filter. Without this MPEG-4
		 * and H.263 decode show visible blocking artifacts at the
		 * 16-pixel macroblock boundaries. Legacy writes
		 * decoder->post_filter.post_filter which defaults to 1
		 * (enabled).
		 */
		vidc_write(core, VIDC_REG_DEC_MPEG4_PP_FILTER, 1);
		return 0;

	case VIDC_CODEC_DIVX311_DEC:
		/*
		 * DivX 3.11 manual resolution override. The bitstream
		 * lacks resolution info, so the host must supply
		 * dimensions before SEQ_HEADER fires. inst->width /
		 * inst->height come from the S_FMT request prior to
		 * STREAMON.
		 *
		 * The legacy code only writes the override for the
		 * VCD_CODEC_DIVX_3 case (vcd_ddl_vidc.c:128) and writes
		 * zeros for other DivX variants - same pattern here.
		 */
		vidc_write(core, VIDC_REG_DEC_DIVX3_WIDTH, inst->width);
		vidc_write(core, VIDC_REG_DEC_DIVX3_HEIGHT, inst->height);
		return 0;

	case VIDC_CODEC_DIVX412_DEC:
	case VIDC_CODEC_DIVX502_DEC:
	case VIDC_CODEC_DIVX503_DEC:
		/* DivX 4/5 carry resolution in their bitstream; clear the
		 * DivX3 override registers in case stale values were left
		 * from a previous DivX3 session. */
		vidc_write(core, VIDC_REG_DEC_DIVX3_WIDTH, 0);
		vidc_write(core, VIDC_REG_DEC_DIVX3_HEIGHT, 0);
		return 0;

	case VIDC_CODEC_VC1_DEC:
	case VIDC_CODEC_VC1_RCV_DEC:
		/*
		 * VC-1 RCV-format streams need a resolution-swap
		 * register write. AP/MP profiles parse from the
		 * bitstream and don't need it.
		 */
		dev_warn_once(core->dev,
			      "VC-1 decode without RCV resolution swap\n");
		return 0;

	default:
		dev_warn_once(core->dev,
			      "unknown codec %d - no codec config applied\n",
			      inst->codec);
		return 0;
	}
}

void vidc_free_buffers(struct vidc_inst *inst)
{
	struct vidc_core *core = inst->core;
	u32 i;

	if (!inst->dpb_y_vaddr)
		return;

	/*
	 * Clear register slots so a subsequent OPEN_CH on the same
	 * channel starts from a clean slate — stale offsets would point
	 * at freed memory and the firmware would happily DMA into it.
	 * Decoder and encoder use different register groups.
	 */
	if (inst->decoder) {
		for (i = 0; i < inst->dpb_count; i++) {
			vidc_write(core, VIDC_REG_DPB_LUMA_BASE + i * 4, 0);
			vidc_write(core, VIDC_REG_DPB_CHROMA_BASE + i * 4, 0);
			if (inst->dpb_mv_size)
				vidc_write(core, VIDC_REG_DPB_MV_BASE + i * 4, 0);
		}
	} else {
		for (i = 0; i < inst->dpb_count; i++) {
			vidc_write(core, VIDC_REG_RECON_LUMA_0 + i * 8, 0);
			vidc_write(core, VIDC_REG_RECON_CHROMA_0 + i * 8, 0);
		}
	}

	dma_free_coherent(core->dev, inst->dpb_y_alloc_size,
			  inst->dpb_y_vaddr, inst->dpb_y_dma_addr);
	inst->dpb_y_vaddr = NULL;
	inst->dpb_y_dma_addr = 0;
	inst->dpb_y_alloc_size = 0;
	inst->dpb_count = 0;
	inst->dpb_y_size = 0;
	inst->dpb_c_size = 0;
	inst->dpb_mv_size = 0;
	inst->dpb_inited = false;
}

void vidc_core_deinit(struct vidc_core *core)
{
	vidc_unload_firmware(core);
	vidc_clk_disable(core);
}

static int vidc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct vidc_core *core;
	struct resource *res;
	int ret;

	core = devm_kzalloc(dev, sizeof(*core), GFP_KERNEL);
	if (!core)
		return -ENOMEM;

	core->dev = dev;
	mutex_init(&core->lock);
	spin_lock_init(&core->irqlock);
	init_completion(&core->fw_status_done);
	init_completion(&core->sys_init_done);

	/*
	 * Set up DMA parameters needed by vb2_dma_contig. Without this,
	 * the first VIDIOC_MMAP from userspace crashes the kernel in
	 * vb2_mmap → dma_get_max_seg_size → NULL deref on dev->dma_parms.
	 * Discovered 2026-05-13 when v4l2-ctl --stream-out-mmap on
	 * /dev/video6 took down the device:
	 *   Unable to handle kernel NULL pointer dereference at 0x000001b8
	 *   PC is at vb2_mmap+0x60/0x2bc
	 *   LR is at v4l2_m2m_fop_mmap+0x3c/0x40
	 *
	 * 32-bit DMA mask is correct for this hardware — VIDC uses
	 * sub-4GB physical addresses (CMA region at 0x7c000000-0x7dffffff).
	 */
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(dev, "failed to set DMA mask: %d\n", ret);
		return ret;
	}
	dev->dma_parms = devm_kzalloc(dev, sizeof(*dev->dma_parms),
				      GFP_KERNEL);
	if (!dev->dma_parms)
		return -ENOMEM;
	dma_set_max_seg_size(dev, DMA_BIT_MASK(32));
	INIT_LIST_HEAD(&core->instances);

	/* Map registers */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	core->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(core->base))
		return PTR_ERR(core->base);

	/* Get IRQ */
	core->irq = platform_get_irq(pdev, 0);
	if (core->irq < 0)
		return core->irq;

	ret = devm_request_irq(dev, core->irq, vidc_isr, IRQF_TRIGGER_HIGH,
			       "vidc", core);
	if (ret) {
		dev_err(dev, "failed to request IRQ: %d\n", ret);
		return ret;
	}

	/* Get clocks */
	core->core_clk = devm_clk_get(dev, "core");
	if (IS_ERR(core->core_clk)) {
		dev_err(dev, "failed to get core clock\n");
		return PTR_ERR(core->core_clk);
	}

	core->iface_clk = devm_clk_get(dev, "iface");
	if (IS_ERR(core->iface_clk)) {
		dev_err(dev, "failed to get iface clock\n");
		return PTR_ERR(core->iface_clk);
	}

	core->axi_clk = devm_clk_get(dev, "axi");
	if (IS_ERR(core->axi_clk)) {
		dev_err(dev, "failed to get axi clock\n");
		return PTR_ERR(core->axi_clk);
	}

	/* Set initial clock rate */
	ret = clk_set_rate(core->core_clk, vidc_clk_rates[2]); /* 96 MHz */
	if (ret) {
		dev_err(dev, "failed to set core clock rate: %d\n", ret);
		return ret;
	}

	/* Get optional power domain regulator */
	core->gdsc = devm_regulator_get_optional(dev, "gdsc");
	if (IS_ERR(core->gdsc)) {
		if (PTR_ERR(core->gdsc) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		core->gdsc = NULL;
	}

	/*
	 * Read SMI reserved-memory region for firmware placement.
	 * The VIDC RISC CPU can only fetch from SMI (0x38000000); firmware
	 * placed in EBI/DRAM is unreachable for the RISC.
	 */
	if (dev->of_node) {
		struct device_node *mem_node;
		struct resource r;

		mem_node = of_parse_phandle(dev->of_node, "memory-region", 0);
		if (mem_node) {
			if (of_address_to_resource(mem_node, 0, &r) == 0) {
				core->fw_phys_base = r.start;
				core->fw_phys_size = resource_size(&r);
				dev_info(dev, "SMI firmware region: 0x%08x size 0x%zx\n",
					 (u32)core->fw_phys_base, core->fw_phys_size);
			} else {
				dev_err(dev, "failed to get SMI memory-region address\n");
				of_node_put(mem_node);
				return -EINVAL;
			}
			of_node_put(mem_node);
		} else {
			dev_err(dev, "no memory-region specified; firmware cannot be placed in SMI\n");
			return -EINVAL;
		}
	}

	/* video-smi: RISC AXI fetch from SMI (critical: without this vote the
	 * MMSS fabric blocks RISC instruction fetches and RISC stalls silently) */
	core->icc_path = devm_of_icc_get(dev, "video-smi");
	if (IS_ERR(core->icc_path)) {
		ret = PTR_ERR(core->icc_path);
		if (ret == -EPROBE_DEFER)
			return ret;
		dev_err(dev, "video-smi ICC path failed (err=%d): RISC will stall on first fetch\n", ret);
		core->icc_path = NULL;
	} else if (core->icc_path) {
		dev_info(dev, "video-smi ICC path acquired OK\n");
	}

	/* video-ebi: RISC access to EBI DRAM (decoded frame buffers) */
	core->icc_ebi_path = devm_of_icc_get(dev, "video-ebi");
	if (IS_ERR(core->icc_ebi_path)) {
		ret = PTR_ERR(core->icc_ebi_path);
		if (ret == -EPROBE_DEFER)
			return ret;
		dev_err(dev, "video-ebi ICC path failed (err=%d)\n", ret);
		core->icc_ebi_path = NULL;
	} else if (core->icc_ebi_path) {
		dev_info(dev, "video-ebi ICC path acquired OK\n");
	}

	/*
	 * Read bandwidth from device tree, with defaults for 1080p video.
	 * Properties: qcom,icc-bw-avg-kbps, qcom,icc-bw-peak-kbps
	 */
	core->icc_bw_avg = VIDC_BW_AVG;
	core->icc_bw_peak = VIDC_BW_PEAK;
	if (dev->of_node) {
		u32 val;

		if (!of_property_read_u32(dev->of_node, "qcom,icc-bw-avg-kbps", &val))
			core->icc_bw_avg = val;
		if (!of_property_read_u32(dev->of_node, "qcom,icc-bw-peak-kbps", &val))
			core->icc_bw_peak = val;
	}

	/* Register V4L2 device */
	ret = v4l2_device_register(dev, &core->v4l2_dev);
	if (ret) {
		dev_err(dev, "failed to register V4L2 device: %d\n", ret);
		return ret;
	}

	/* Register decoder video device */
	ret = vidc_dec_register(core);
	if (ret) {
		dev_err(dev, "failed to register decoder: %d\n", ret);
		goto err_v4l2_unregister;
	}

	/* Register encoder video device */
	ret = vidc_enc_register(core);
	if (ret) {
		dev_err(dev, "failed to register encoder: %d\n", ret);
		goto err_dec_unregister;
	}

	platform_set_drvdata(pdev, core);

	pm_runtime_enable(dev);

	dev_info(dev, "Qualcomm VIDC 1080p driver probed\n");

	return 0;

err_dec_unregister:
	vidc_dec_unregister(core);
err_v4l2_unregister:
	v4l2_device_unregister(&core->v4l2_dev);
	return ret;
}

static void vidc_remove(struct platform_device *pdev)
{
	struct vidc_core *core = platform_get_drvdata(pdev);

	pm_runtime_disable(core->dev);
	vidc_enc_unregister(core);
	vidc_dec_unregister(core);
	vidc_core_deinit(core);
	v4l2_device_unregister(&core->v4l2_dev);
}

static int vidc_runtime_suspend(struct device *dev)
{
	struct vidc_core *core = dev_get_drvdata(dev);

	vidc_clk_disable(core);

	if (core->icc_path)
		icc_set_bw(core->icc_path, 0, 0);
	if (core->icc_ebi_path)
		icc_set_bw(core->icc_ebi_path, 0, 0);

	if (core->gdsc)
		regulator_disable(core->gdsc);

	/*
	 * Clocks are cut at this point regardless of whether GDSC is
	 * managed here. The RISC is frozen mid-execution; its internal
	 * state on clock-restore is undefined. Always mark fw_running=false
	 * so the next vidc_runtime_resume unconditionally runs vidc_boot_firmware
	 * (hw_reset + SYS_INIT), giving the RISC a clean slate every time.
	 *
	 * Previously this was gated on core->gdsc != NULL (GDSC drop assumed
	 * to be the only state-wiping event), but on tenderloin the video-codec
	 * DT node has no gdsc-supply, so core->gdsc is NULL and fw_running
	 * was never cleared — the second OPEN_CH received cmd=0 (RESP_EMPTY)
	 * from the RISC because it had been interrupted mid-cleanup.
	 */
	core->fw_running = false;

	return 0;
}

static int vidc_runtime_resume(struct device *dev)
{
	struct vidc_core *core = dev_get_drvdata(dev);
	int ret;

	if (core->gdsc) {
		ret = regulator_enable(core->gdsc);
		if (ret)
			return ret;
	}

	if (core->icc_path) {
		dev_info(dev, "voting video-smi bw: avg=%u peak=%u kbps\n",
			 core->icc_bw_avg, core->icc_bw_peak);
		ret = icc_set_bw(core->icc_path, core->icc_bw_avg, core->icc_bw_peak);
		if (ret) {
			dev_err(dev, "failed to set video-smi bandwidth: %d\n",
				ret);
			goto err_gdsc;
		}
		dev_info(dev, "video-smi bandwidth vote ok\n");
	} else {
		dev_err(dev, "video-smi ICC path is NULL — RISC fetch will be blocked\n");
	}
	if (core->icc_ebi_path) {
		ret = icc_set_bw(core->icc_ebi_path, core->icc_bw_avg, core->icc_bw_peak);
		if (ret) {
			dev_err(dev, "failed to set video-ebi bandwidth: %d\n",
				ret);
			goto err_icc_smi;
		}
	}

	ret = vidc_clk_enable(core);
	if (ret)
		goto err_icc;

	/*
	 * Re-boot the firmware if a GDSC drop in the previous suspend
	 * wiped the boot state. No-op if fw_running was already true
	 * (e.g. first resume after probe, or quick suspend that didn't
	 * actually drop the regulator).
	 *
	 * If fw_loaded is false (probe hasn't yet called
	 * vidc_load_firmware), this is also a no-op via the !fw_loaded
	 * early-return in vidc_boot_firmware().
	 */
	ret = vidc_boot_firmware(core);
	if (ret && core->fw_loaded) {
		dev_err(dev, "firmware boot failed on resume: %d\n", ret);
		goto err_clk;
	}

	return 0;

err_clk:
	vidc_clk_disable(core);
err_icc:
	if (core->icc_ebi_path)
		icc_set_bw(core->icc_ebi_path, 0, 0);
err_icc_smi:
	if (core->icc_path)
		icc_set_bw(core->icc_path, 0, 0);
err_gdsc:
	if (core->gdsc)
		regulator_disable(core->gdsc);
	return ret;
}

static const struct dev_pm_ops vidc_pm_ops = {
	SET_RUNTIME_PM_OPS(vidc_runtime_suspend, vidc_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				pm_runtime_force_resume)
};

static const struct of_device_id vidc_of_match[] = {
	{ .compatible = "qcom,msm8660-vidc" },
	{ .compatible = "qcom,apq8060-vidc" },
	{ },
};
MODULE_DEVICE_TABLE(of, vidc_of_match);

static struct platform_driver vidc_driver = {
	.probe = vidc_probe,
	.remove = vidc_remove,
	.driver = {
		.name = "qcom-vidc",
		.of_match_table = vidc_of_match,
		.pm = &vidc_pm_ops,
	},
};

module_platform_driver(vidc_driver);

MODULE_DESCRIPTION("Qualcomm VIDC 1080p Video Codec driver");
MODULE_LICENSE("GPL v2");
MODULE_FIRMWARE(VIDC_FW_NAME);
