# AR6003 mailbox CMD53 trace — legacy vs mainline

**Date:** 2026-05-21
**Goal:** Find why mainline mmci-pl18x on sdcc4 hangs mid-stream during the
post-BMI HTC body read (128 B FIXED-address from mbox 0x800), while legacy
2.6.35-palm-tenderloin msm_sdcc on the same hardware completes it cleanly.

## How the legacy capture was produced

1. Added `AR6K_TRACE_CMD53_MBOX(cmd)` filter + `msmsdcc_ar6k_dump()` helper
   to `drivers/mmc/host/msm_sdcc.c` in
   `/home/herrie/webos/touchpad-kernel/webos-linux-kernel-touchpad`. The filter
   matches `cmd->opcode == SD_IO_RW_EXTENDED` AND addr ∈ `[0x800, 0xfff]` so
   only the AR6003 mailbox traffic is logged (eMMC on sdcc1 unaffected).
2. Trace points: `pre-data` (entry to `msmsdcc_start_data`), DMA-path
   `datactrl-stashed`, PIO-path `post-dctrl-pio`, one-shot `in-irq` raw status
   dump per request, dedicated `STARTBITERR observed` line (since MASK0
   doesn't enable bit 9), and final `request_end` dump with
   `data_xfered/xfer_size` and `DATACNT`.
3. Built with `/opt/PalmPDK/arm-gcc/bin/arm-none-linux-gnueabi-gcc` 4.3.3
   against `tenderloin_defconfig`. zImage packed as single-image uImage
   (load/entry `0x40208000`), deployed as `/uboot/uImage.webOSdebug` —
   `/uboot/uImage.webOS -> uImage-2.6.35-palm-tenderloin` left untouched as
   fallback. `CONFIG_MODVERSIONS` off, so the production `ar6000.ko` on
   `store-root` loads against the rebuilt kernel.
4. Booted via moboot menu → `webOSdebug`. dmesg pulled via novacom into
   `webos-debug-dmesg-full.log`; the `AR6K-DBG` subset is in
   `webos-debug-ar6k-only.log`. Debug zImage and uImage stashed alongside
   so the next reboot doesn't require a rebuild.

## Key findings

**1. STARTBITERR never observed on legacy.** `grep "STARTBITERR observed"
webos-debug-ar6k-only.log` → 0. The AR6003 clocks the full transfer
cleanly on legacy. The mainline failure mode (DATACNT=24, RXFIFOFULL=1,
STARTBITERR=1 mid-stream) is **not** an inherent chip behaviour — it is
introduced by something in the mainline path.

**2. Legacy uses DMA for all mailbox transfers ≥ 128 B.** Datactrl-stashed
values from the trace:

| Operation     | dctrl value | blksz field | DMAENABLE bit |
|---------------|-------------|-------------|---------------|
| WR 128/256/.. | 0x809/1009/1809/2009 | 128/256/384/512 | 1 |
| RD 128/256/.. | 0x80b/100b/180b/200b | 128/256/384/512 | 1 |
| RD 24B INCR (lookahead poll) | 0x183 | 24 | **0** |

So even on legacy, the small status-register lookahead reads at `0x400`
stay PIO; the larger mailbox reads at `0x800` go DMA. Mainline DT does
provide `dmas = <&adm_dma1 5>` and `qcom,sdcc-crci = <5>` on sdcc4, and
the live MMCIDATACTRL = `0x80b` on the mainline-side hang likewise shows
DMAENABLE = 1 — i.e. mainline is *also* taking the DMA path. The hang is
**not** because mainline went PIO and legacy went DMA.

**3. Legacy never emits CMD53 BLOCK-mode for ≤ 11 blocks.** Distinct `arg`
values seen in the trace:

```
arg=0x10100000 / 0x10100080 / 0x10100100 / 0x10100180   block=0 (byte mode)
arg=0x1810000c / 0x18100015 / 0x18100030 / 0x1810003c   block=1, blocks∈{12,21,48,60}
arg=0x18100048 / 0x18100060                              block=1, blocks∈{72,96}
```

Specifically the body-read-shaped argument `0x18100001` (block-mode, 1
block, addr=0x800) is **never** issued by legacy. Single 128 B reads use
byte-mode CMD53 with byte count = 128 (`arg=0x10100080`, 105 occurrences
in the trace).

The legacy kernel's `sdio_io_rw_ext_helper` reaches byte mode for these
because `sdio_max_byte_size(func)` returns 512 (no
`MMC_QUIRK_BLKSZ_FOR_BYTE_MODE`, so `mval = min(host_max, func->max_blksize)
= 512`), so the `size > sdio_max_byte_size(func)` gate is false for any
read ≤ 512 B. Mainline does the same calculation, so on paper mainline
should *also* be issuing byte-mode 128 B CMD53. **This needs direct
confirmation from a mainline cmd.arg log** — open question below.

## Open question — RESOLVED

Diagnostic `dev_info` in `mmc_io_rw_extended` (commit b07948cd7645, gated
on fn==1 && addr ∈ [0x800,0xfff]) captured the exact mainline CMD53
emission for the failing transfer. The last line before the hang:

```
[  442.706868][ T2017] mmc mmc1:0001: AR6K-MAINLINE CMD53 arg=0x10100080
  RD addr=0x800 blocks=0 blksz=128 mode=BYTE incr=0
```

This is **byte-identical** to the legacy capture (`arg=0x10100080`, 105
occurrences). The SDIO command layer is innocent.

Mainline RD breakdown (`mainline-debug-cmd53.log`):

```
   9   RD addr=0x800 blksz=4   incr=1   (success — status polls)
   1   RD addr=0x800 blksz=8   incr=1   (success — status polls)
   1   RD addr=0x800 blksz=128 incr=0   (HANG — the body read)
```

Mainline WRs are all byte-mode INCR (`addr=0xfcc..0xffc`, BMI writes),
all succeed. The body read is the very first FIXED-address read mainline
ever issues to the mailbox.

## Updated suspect ranking — SMOKING GUN (2026-05-21 post-devmem)

**qcom_adm channel 5 was NEVER ARMED for the CMD53 read.**

Live devmem of ADM1 (base 0x18420000) channel 5 register block during the
hang state (commit 16ade9ee6dce):

```
0x18420a80 (CMD_PTR):   0x00000000
0x18420a84 (RSLT_0):    0x00000000
0x18420a88 (RSLT_1):    0x00000000
0x18420a8c (RSLT_2):    0x00000000
0x18420a90 (CONFIG):    0x00000000
0x18420a94 (STATUS):    0x00000000
```

All zeros. Channel 0 had stale descriptor 0x93B06BFD, channel 2 (eMMC)
also idle. The qcom_adm driver either:
1. Never received the DMA descriptor submit from mmci, or
2. Received it but silently failed to program the channel hardware

This explains RXFIFOFULL perfectly: SDCC DPSM started the data phase,
the AR6003 chip began sending bytes, RX FIFO filled to 64 B (one
FIFOSIZE), FLOWENA stopped SDCLK to prevent overrun, but NO DMA CHANNEL
WAS RUNNING to drain the FIFO. SDCLK stays stopped indefinitely, chip
times out on its mailbox-output window, asserts STARTBITERR on the data
lines, and the request hangs forever (no DATAEND, no DATATIMEOUT IRQ
because the timeout counter also freezes when clock stops).

**Next step:** Instrument mmci_dmae_submit + qcom_dma_issue_pending to
determine if the descriptor even reaches dmaengine_submit, and whether
issue_pending is ever called. Commit 77b1bd8f4185 adds dev_info traces
for this.

Legacy msm_dmov_enqueue_cmd_ext obviously programs the channel correctly
— devmem comparison against webOS mid-transfer would show non-zero
CMD_PTR. But mainline never gets that far.

## Confirmed-NOT (so future investigators don't re-tread)

- ❌ Byte-vs-block CMD53 mode (mainline and legacy both byte-mode)
- ❌ FIXED vs INCR address calculation (both use `addr=0x800`, no
  WR-style adjustment on read; that was falsified earlier in
  `5be9a7a3b8f9`)
- ❌ PIO vs DMA (both use DMA, both set DATACTRL DMAENABLE=1)
- ❌ Block-size programming (both program `cur_blksize=128` on the
  SDIO function, both `data->blksz=128` to the SDCC)
- ❌ DPSM `DATALENGTH` value (both 128)
- ❌ HIF_MBOX_BLOCK_SIZE (128 in both ath6kl variants)
- ❌ ADM channel hardware programming (devmem 2026-05-21: channel 5
  CMD_PTR=0, never armed — issue is in the mmci→qcom_adm submit path,
  not the channel config itself)

## Next experiment options

**A. Force PIO on sdcc4** by stripping `dmas`/`dma-names` from the DT
node and rebuilding. If PIO works, that locks qcom_adm as the culprit
and we focus the fix there. If PIO also hangs, the issue is below the
DMA engine entirely (host bus arbitration, CRCI, or chip state).

**B. devmem the qcom_adm channel registers** (channel 5 on adm_dma1)
just before the body read fires. Compare against a webOS devmem snapshot
of the equivalent channel right before its 128 B mailbox read. Any
register difference is a candidate.

**C. Instrument `qcom_dma_start`/`qcom_dma_issue_pending` in mmci** to
log channel state right before the DMA submit + right after, just for
sdcc4 mailbox reads. Pair with `AR6K-MAINLINE` lines to bracket the
failure.

**D. Surgery hypothesis from memory `project_tenderloin_dma_concurrent`:**
all four prior DMA-concurrent attempts on tenderloin failed; only
PIO+PIO was stable. May indicate the qcom_adm contention is fundamentally
unworkable for sdcc4 on this SoC and the right answer is to push sdcc4
to PIO permanently (matching the eMMC-side conclusion from
`project_adm_irq_completion_combo`).

Recommend B first (cheapest, no rebuild) then A if B is inconclusive.

## Suspect list (in order of likelihood given the legacy evidence)

1. **qcom_adm channel programming for sdcc4 differs from `msm_dmov`.**
   Legacy uses `msm_dmov_enqueue_cmd_ext` on channel 21 (CRCI 5); mainline
   uses `qcom_adm` on `adm_dma1 5`. CRCI flow control or burst length
   could be programmed differently — and the live state (`RXFIFOFULL=1`,
   `DATACNT=24` stuck) is consistent with the ADM channel never draining
   the SDCC RX FIFO. `project_adm_irq_completion_combo` already documents
   that mainline's ADM completion path is ~3× heavier than webOS's, so
   the channel mechanics are known-divergent.
2. **AR6003 chip-side state.** Either an SDIO function-block-size
   handshake or a CCCR register write that legacy ar6000.ko does
   somewhere between the lookahead poll and the body read, but mainline
   ath6kl_sdio doesn't. Less likely given both call `sdio_readsb` and
   set block size identically, but not impossible.
3. **DPSM bit-encoding difference.** Both legacy and mainline ended up
   programming DATACTRL=0x80B in the trace, so this is the weakest
   suspect; included only because the live MMCISTATUS showed STARTBITERR
   which is a data-line-level event, not a host-FIFO event.

## Artefacts

- `webos-debug-dmesg-full.log` — full novacom dmesg from the legacy debug
  boot (1149 lines).
- `webos-debug-ar6k-only.log` — `AR6K-DBG` subset (1133 lines).
- `zImage-legacy-debug.bak` — built zImage. Repack with
  `mkimage -A arm -O linux -T kernel -C none -a 0x40208000 -e 0x40208000
  -n "Linux-2.6.35-palm-tenderloin-ar6kdbg" -d zImage-legacy-debug.bak
  uImage.webOSdebug` if needed.
- `uImage.webOSdebug` — packed uImage; identical to what was scp'd to
  `/uboot/uImage.webOSdebug` on the device.

## Reproducing the legacy capture

```
ssh -p 22 root@172.16.42.2 'mount -o remount,rw /uboot'
scp -P 22 reports/ar6003-cmd53-trace/uImage.webOSdebug \
    root@172.16.42.2:/uboot/uImage.webOSdebug
ssh -p 22 root@172.16.42.2 'mount -o remount,ro /uboot'
# Set moboot.next or pick at the moboot menu, then reboot.
novacom run file://bin/dmesg > reports/ar6003-cmd53-trace/webos-debug-dmesg-N.log
```

(`/uboot/uImage.webOS` is the production symlink; leave that alone.)
