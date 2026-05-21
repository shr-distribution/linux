# CE2 AES multi-block cap on MSM8660 — brief for Gemini

## TL;DR

Trying to lift a multi-block AES cap on Qualcomm MSM8660 / APQ8060 Crypto
Engine 2 (CE2) running mainline Linux 6.18.  The HP TouchPad ships
this SoC.  Output ciphertext matches openssl reference for the first
2-4 AES blocks then diverges; longer transfers either silently corrupt
or trigger `DIN_ERR | SW_ERR` in `CE2_REG_STATUS`.

Same silicon ran webOS 3.0 (msm-3.0.8 kernel) fine with AES-CBC at
several MB/s.  We have the legacy webOS driver source, a Samsung
MSM8660 qce.ko (decompiled with Ghidra), and HTC sbl3 disassembly as
references.  None of the legacy patterns I've tried have lifted the
cap.  Asking for a fresh angle.

## Hardware overview

- SoC: Qualcomm APQ8060 (= MSM8660 minus modem), dual Scorpion @ 1.5 GHz
- CE2 MMIO base: `0x18500000`
- DMA: ADM (Application Data Mover) controller `0x18320000`, channels
  2 (CE_IN, CRCI 4) and 3 (CE_OUT, CRCI 5)
- Data port: `CE2_REG_DATA_SHADOW0` at offset `+0x8000` (FIFO peephole)

CE2 register map (relevant):

```
0x000  DATA_IN              0x024  CONFIG
0x010  DATA_OUT             0x030  SEG_CFG
0x020  STATUS               0x034  ENCR_SEG_CFG
0x03C  SEG_SIZE             0x040  GOPROC
0x044  ENGINES_AVAIL        0x050..0x064  DES_KEY0..5
0x070..0x07C  CNTR0_IV0..CNTR3_IV3
0x080  CNTR_MASK            0x100..0x13C  AUTH_IV0..15
0x200..0x2EC  AES_RNDKEY0..59
0x8000  DATA_SHADOW0  (FIFO port; same physical FIFO seen here)
```

STATUS register layout (bit numbers):
- 31-28 CORE_REV
- 24-22 DOUT_SIZE_AVAIL (dwords)
- 21-19 DIN_SIZE_AVAIL (dwords)
- 18 ACCESS_VIOL, 17 SEG_CHNG_ERR, 16 CFG_CHNG_ERR
- 15 DOUT_ERR, 14 DIN_ERR
- 13 LOCKED, 12-10 CRYPTO_STATE (0=IDLE, 1=LOCKED, 3=GO, 4=PROCESSING,
  5=FINAL_READ, 6=CTXT_CLEARING, 7=UNLOCKING)
- 9 ENCR_BUSY, 8 AUTH_BUSY
- 7 DOUT_INTR, 6 DIN_INTR, 5 AUTH_DONE_INTR, 4 ERR_INTR
- 3 DOUT_RDY, 2 DIN_RDY, 1 AUTH_DONE, 0 SW_ERR

SEG_CFG bits:
- 25 F8_KEYSTREAM_EN, 24 F9_DIR, 23 F8_DIR
- 22 USE_HW_KEY
- 21-20 CNTR_ALG
- 19 CLR_CNTXT, 18 LAST, 17 FIRST, 16 ENCODE
- 15-14 AUTH_POS (0 BEFORE, 1 AFTER)
- 13-11 AUTH_SIZE, 10-9 AUTH_ALG
- 8-6 ENCR_MODE (0=ECB,1=CBC,2=CTR,3=CCM)
- 5-3 ENCR_KEY_SZ (0=AES128, 1=AES192, 2=AES256, 4=DES, 5=3DES)
- 2-0 ENCR_ALG (1=DES, 2=AES, 4=KASUMI, 5=SNOW3G)

## Symptoms

Test tools:
- `qce-cipher-test`: NIST/FIPS-197 vectors via AF_ALG (sizes ≤32 B)
- `qce-streamenc`: stream a file through `cbc-aes-qce` and compare to
  `openssl enc -aes-128-cbc -nopad`

Best stable configuration (pre-step-4):
- All 12 NIST vectors pass (AES-128 ECB/CBC/CTR, AES-256-CBC, DES-CBC,
  3DES-CBC, encrypt + decrypt for each)
- `qce-streamenc` 32 B / 48 B input: full match
- 64 B input: blocks 1-3 match, block 4 differs
- ≥80 B input: blocks 1-4 match, block 5+ wrong (stale shadow /
  passthrough)

Symptom invariants across burst sizes, descriptor topologies, and
key-schedule strategies:
- DES (8-byte block) handles up to 32 KB per op fine
- AES caps at 4 blocks (64 B) regardless of key size (128/192/256) or
  mode (ECB/CBC/CTR)
- Symptoms past the cap: STATUS = `0x10205205` (DIN_ERR=1, SW_ERR=1,
  CRYPTO_STATE=PROCESSING, ENCR_BUSY=1, DIN_RDY=1) when the engine
  errors out, or stale-shadow data with no error flag when the data
  silently corrupts.

## What we've tried

Everything below is on the SAME silicon that runs webOS fine.

### Key handling
1. **Raw key write** (engine `fastaes=1` internal expansion): writes only
   `keylen / 4` dwords starting at `AES_RNDKEY0`.  NIST passes, bulk caps.
2. **Full FIPS-197 expansion** (Samsung qce.ko pattern, 60 dwords to
   RNDKEY0..59).  Initial attempt seeded wrong (LE seed via
   `aes_expandkey()` vs BE seed Samsung uses); fixed by BE-packing the
   input key first so the LE-read inside `aes_expandkey` reconstitutes
   the BE-form seed.  Verified bit-for-bit match with Samsung's
   schedule.  NIST still passes.  **Bulk cap unchanged.**

Conclusion: the cap is NOT in the engine's key expander state machine.

### DMA pacing
3. **Burst-size sweep** {8, 16, 32, 64, 128, 192, 256} bytes — same cap
   at all sizes.  With non-FC ADM (the default) ADM blasts data
   regardless of CRCI; with FC enabled different failure mode but
   never lifts the cap.
4. **CRCI flow control** via `device_fc=true` in `dma_slave_config`.
   Without it, qcom_adm silently routes through the non-flow-controlled
   path that ignores crci AND swap flags.  With it, engine times out
   waiting for CRCI handshakes that never fire (chicken-and-egg with
   GOPROC ordering).
5. **GOPROC reorder**: arm ADM channels first (`issue_pending`), THEN
   write GOPROC, so the engine asserts DIN_RDY into a ready ADM.
   Eliminates the deadlock.  Cap still present, just appears as
   DIN_ERR mid-stream instead of timeout.
6. **CMD_MODE_SG vs CMD_TYPE_BOX**.  Legacy webOS qce.c uses ADM
   scatter-gather mode (cmd bits 0-2 = 001) instead of BOX (= 011).
   I patched `qcom_adm.c` to support SG mode with src_dscr /
   dst_dscr pointing at single-entry `adm_sg_desc` arrays — cmd word
   `0x8000c201` (LC | DST_SWAP_BYTES | DST_SWAP_SHORTS | DST_CRCI(4) |
   MODE_SG) matches webOS byte-for-byte.  Bulk cap is now 3 blocks
   (48 B) instead of 4 (64 B).  Marginal change.  ≥1 KB transfers
   produce zero-byte output (DMA times out with DIN_ERR + SW_ERR).

### Byte ordering
7. **Software byte-swap** before/after DMA (cpu_to_be32 each dword) —
   the original approach.  Matches webOS's `_byte_stream_to_net_words`
   for keys/IVs but NOT for data (webOS data goes through ADM hw swap).
8. **ADM hw swap** via `CMD_DST_SWAP_BYTES | CMD_DST_SWAP_SHORTS` (or
   SRC equivalents for DEV_TO_MEM).  Patched qcom_adm to plumb these
   through `qcom_adm_peripheral_config::cmd_flags`.  Functionally
   equivalent end-to-end byte value at the engine port either way.

### Engine reset
9. Per-op `reset_control_assert/deassert` on `GCC_CE2_RESET` (1ms +
   1ms settle).  Required — without it engine wedges after ~5 ops.
10. Per-op SW_RST pulse via CONFIG (bit 0): 10us + 10us.  Required at
    one point for 3DES decrypt cold-start; later made redundant by
    dropping CLR_CNTXT.

### SEG_CFG variations
11. **CLR_CNTXT** (bit 19 of SEG_CFG): tested both ways.  Required OFF
    for 3DES to work across cold-start.  Webos legacy doesn't set it
    either.  Caps AES at 4 blocks when set.
12. **AUTH_POS** (bit 14): required for encrypt direction.  Without it,
    ECB decrypt returns input unchanged, CTR returns garbage.
13. **CTR mode forced ENCODE=1 + AUTH_POS=1**: CTR is symmetric, but
    ENCODE=0 on CE2 flips the engine to `AES_decrypt(input) XOR counter`
    instead of `input XOR AES_encrypt(counter)` — empirically broken
    so we force encrypt-direction always.
14. **CNTR_MASK = 0xffff** for all AES (not just CTR) — matches webOS
    line 1503 and Samsung _ce_setup.  No change to cap behaviour.

## Reference implementations studied

1. **webOS qce.c** (msm-3.0.8 era): uses CMD_MODE_SG with byte+short
   hw swap, ADM_DESC_LAST terminator.  Engine setup: clk_enable per op
   (not reset), CONFIG = 0x78 once at init, SW expansion when
   `fastaes=0`, raw key when `fastaes=1`.  Path: `_ce_setup_cipher()`
   in `drivers/crypto/msm/qce.c`.  **This works for arbitrary lengths
   on the same silicon.**

2. **Samsung SGH-I727 qce.ko** (Ghidra decompile of MSM8660 native
   driver, codeaurora msm-3.0.8): writes full FIPS-197 expanded
   schedule to AES_RNDKEY0..59 regardless of AES_SEL_FAST.  Per-op
   clk_enable, no reset, no SW_RST.  Static DMA buffer pre-allocated
   at open() with two SG lists (128 × 8 B entries each).

3. **HTC sbl3** (MSM8x60 bootloader, disassembly at `sbl3.disasm:5f30c`):
   pre-expands AES round-key schedule in 60-word loop.  Uses PIO via
   DATA_IN with DIN_RDY polling (no ADM).

## The key architectural question

webOS + Samsung both target the SAME MSM8660 silicon and both achieve
multi-block AES.  Both use `CMD_MODE_SG` for the ADM.  Mainline
qcom_adm.c emits `CMD_TYPE_BOX` (or `CMD_TYPE_SINGLE`).

I patched qcom_adm to support SG mode and the cmd word matches webOS
exactly (`0x8000c201` for CE_IN, `0x80001829` for CE_OUT).  But the
cap is still there — only 3 blocks instead of 4.

**What am I missing?**

Possible hypotheses I haven't validated:
- Engine internal FIFO size (some docs suggest 8 dwords = 32 B; with
  data + control bytes maybe effective payload capacity is 4 blocks?)
- ADM scheduler-level timing requirement (the ADM has multiple
  internal queues; webOS may be using a specific cmd_ptr chaining
  pattern we're not replicating)
- CE2's `BAM_IO_REG` (?)  — unmapped in our driver
- A specific CONFIG bit we have wrong (we set bits 3-6 = interrupt
  masks + clear bits 13-15 = high-speed enables; webOS does the same)
- Engine clock rate.  webOS uses `clk_set_rate` at init; we keep the
  default from the clock controller.  CE2 might need a specific rate
  for sustained throughput.
- Engine voltage / power domain — there might be a regulator we're
  not enabling for full-rate operation.
- `qcom_adm_peripheral_config::mux` — we always set it to 0; some
  peripherals need a non-zero mux.  CE2 might.

## Code locations

- Driver: `drivers/crypto/qce/common.c` — function
  `qce_ce2_pio_run_skcipher` (cipher op entry) and
  `qce_ce2_dma_inout_cipher` (DMA setup).
- ADM driver: `drivers/dma/qcom/qcom_adm.c` — function
  `adm_prep_slave_sg` and (newly added) `adm_build_sg_mode`.
- Public header: `include/linux/dma/qcom_adm.h` — flags
  `QCOM_ADM_CMD_FLAG_{SWAP_BYTES,SWAP_SHORTS,MODE_SG}`.
- Register definitions: `drivers/crypto/qce/regs-ce2.h`.

Git branch: `tenderloin/6.18/upstream-patches`.

## Concrete asks for Gemini

1. **What FIFO geometry does the MSM8x60 CE2 actually have?**
   Specifically the DIN/DOUT FIFOs that DATA_SHADOW0 maps onto.
   The Qualcomm AP Crypto Engine Architecture doc references the
   following but I haven't found a public copy:
   - "CE 2.5" with 8-dword DIN/DOUT
   - "CE 4.0" with 32-dword
   This is what determines the per-CRCI-burst max safely deliverable.

2. **Is there a known programming order for "long" (> FIFO) AES ops?**
   E.g., does the engine need a specific drain-and-refill cycle
   between bursts that we're skipping?  Some Qualcomm engines have an
   "intermediate processing" state that needs driver intervention.

3. **CMD_MODE_SG semantics**: Does the ADM walk the SG list with one
   CRCI handshake per descriptor, or just ONE handshake for the whole
   command?  Legacy webOS dma.h marks SG mode "untested" — but their
   qce.c uses it successfully.  Suggests it's well-tested in practice;
   we just don't know if it's per-descriptor pacing or not.

4. **STATUS = 0x10205205** during a 64+ B AES-CBC: with DIN_ERR=1 +
   SW_ERR=1 + CRYPTO_STATE=PROCESSING + DIN_RDY=1, what's the canonical
   recovery?  Reset and retry?  Or is there a specific clear-error
   register write?

5. **Is there a `BAM_DESC_FIFO_LWM_REG` or similar low-water-mark
   register on CE2** that paces DIN_RDY assertion rate?  webOS might
   be programming this.

6. **clk_set_rate on CE2 core clock**: do we need to set it to a
   specific value for multi-block AES?  Default is "whatever the
   bootloader left it at".

## Files attached / available

- `/tmp/qce-test/qce-cipher-test.c` — NIST vector test source
- `/tmp/qce-test/qce-streamenc.c` — bulk match test source
- `/tmp/samsung-i727/qce.ko` — Samsung MSM8660 binary
- `/tmp/ghidra-qce/qce_decompiled.c` — full Ghidra decompile of Samsung
- `reports/ce2-investigation/samsung-qce-deep-dive.md` — earlier
  analysis covering Samsung's _ce_setup function
- `reports/ce2-investigation/htc-sbl3-ce2-pio-sequence.md` — HTC sbl3
  PIO-DATA_IN reference
- Legacy webOS kernel: `/home/herrie/webos/touchpad-kernel/webos-linux-kernel-touchpad/drivers/crypto/msm/qce.c`
