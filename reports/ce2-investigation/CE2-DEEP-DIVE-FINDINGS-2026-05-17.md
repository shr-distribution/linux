# CE2 SHA Wedge Deep Dive — 2026-05-17

Multi-source comparison of CE2/QCE hash sequences across webOS, mako, HTC sbl3
bootloader, Sony MSM8930, mainline qce/v5, and qce40/qce50 (CE3/CE5
generations), to find the root cause of the back-to-back SHA256 wedge.

## Wedge symptom

Per-op PIO hash via writel to DATA_IN (+0x000), with byte-swap fix in place
(commit `bf401e78a607`):

1. SHA1 ops chain reliably (3+ ops back-to-back, any SEG_SIZE).
2. First SHA256 after a SHA1 chain: OK.
3. **Second SHA256 op (back-to-back): wedges.** STATUS=0x10201104
   (state=PROCESSING, AUTH_BUSY=1, DIN_RDY=1). AUTH_DONE never fires.
4. After wedge, all subsequent hash ops fail forever. SW_RST does not recover.

Isolated experiments:
- 1 SHA256 op alone: OK.
- 2 SHA256 ops: both OK.
- 3 SHA256 ops: 2 OK, **3rd wedges**.
- Mode 5 (3 SHA1 → 2 SHA256, mirrors full test): full test reproduces the wedge
  exactly at op 5 (second SHA256).

The pattern is: after 2 consecutive SHA256 ops via PIO readback, the 3rd wedges
in PROCESSING state.

## Cross-kernel sequence comparison

| Step | webOS (8x60) | mako (8960) | HTC sbl3 | qce/v5 mainline | Our PIO |
|---|---|---|---|---|---|
| Per-op SW_RST | **No** | **No** | No | No | **Yes** |
| Clear STATUS=0 | No | No | No | yes (cfg) | Yes |
| AUTH_IV0..N | yes | yes | yes | yes | yes |
| Zero AUTH_IV5..15 | **No** | **No** | No | No | **Yes** |
| AUTH_BYTECNT0/1 | yes | yes | yes (only first) | yes | yes |
| AUTH_BYTECNT2/3 | **No** | **No** | No | CMAC only | **Yes** |
| SEG_CFG order | After IV | After IV | After BYTECNT | Late | After STATUS |
| AUTH_SEG_CFG | After SEG_CFG | After SEG_CFG | After SEG_CFG | After IV | After SEG_CFG |
| SEG_SIZE | Last | Last | Last | Last | Mid |
| CONFIG rmw | **Probe only** | **Probe only** | Probe only | Probe only | **Every op** |
| GOPROC trigger | After SEG_SIZE | After SEG_SIZE | After all | After CONFIG | Last |
| **Data feed** | **ADM DMA box → DATA_SHADOW0 +0x8000 CRCI 4** | same | PIO writel DATA_IN | DMA | PIO writel DATA_IN |
| **Result read** | **ADM DMA SINGLE ← AUTH_IV0 +0x100 CRCI 15** | same | PIO readl | DMA | **PIO readl (no CRCI handshake)** |

Things we do that **nobody else does**:
1. Per-op SW_RST + CONFIG rewrite
2. Zero AUTH_IV5..15 before writing the algorithm IVs
3. Write AUTH_BYTECNT2/3 (only 0/1 are used for 64-bit message length)

## Root cause hypothesis (from the deep dive)

The CE2 state machine after AUTH_DONE transitions through:
`PROCESSING → FINAL_READ → CTXT_CLEARING → UNLOCKING → IDLE`.

**The CRCI 15 (`CE_HASH_CRCI`) handshake on the DMA read of AUTH_IV0 is what
drives the FINAL_READ → CTXT_CLEARING transition.** Without that handshake
firing, the engine's internal hash-state buffers aren't fully cleared. Two
successive ops survive because internal state has some slack; the third
op tips into a wedge.

Our PIO `readl()` of AUTH_IV0..N reads the digest correctly but does **not**
fire CRCI 15. The engine never sees "host has consumed the digest" via the
hardware handshake.

Additional concern: writing zeros to AUTH_IV5..15 while the engine is in
CTXT_CLEARING/UNLOCKING (states 6/7) may itself corrupt the internal cleanup,
since those registers are part of the context block being purged.

## What other vendors don't do

- **HTC sbl3** only hashes once per boot (signature verification). The
  back-to-back code path was never exercised in HTC firmware.
- **Samsung MSM8x60 downstream** drivers (Galaxy S2 family, Tab variants):
  not found on GitHub — those vendors stripped the qce driver in favor of
  TZ-bound `qseecom` user-space crypto.
- **No GitHub source** ever shipped a working PIO-only chained SHA path for
  CE2. Every working reference uses ADM DMA for both data feed and digest
  readout, on a **single channel walking two chained descriptors**.

## Smoking-gun details from sources

1. **webOS qce.c::_setup_cmd_template (lines 1090-1170)** sets up
   `cmd_list_ce_in` with two descriptors: BOX (data → DATA_SHADOW0 with
   CRCI 4) followed by SINGLE (AUTH_IV0 → result_buf with CRCI 15), tagged
   CMD_LC (last command). Both on the same ADM channel walking forward.
   This is the canonical pattern.
2. **mako qce40 / qce50** atomically push the whole register block via DMA
   before GOPROC. STATUS/SEG_CFG/IVs land together — no interleaved CPU
   reads of stale engine state.
3. **qce50 SHA path** never zeros AUTH_IV5..15. Only AES-CMAC explicitly
   zeros AUTH_IV+AUTH_KEY+BYTECNT0 as part of the DMA-pushed buffer.
4. **CE2 GOPROC has only `CRYPTO_GO` bit 0** (per `qcryptohw_30.h`).
   `RESULTS_DUMP` bit 2 only exists in CE5+ (`qcryptohw_50.h`). So we
   cannot manually trigger FINAL_READ from PIO — it must come from the
   CRCI 15 DMA handshake.

## Path forward

### Path B (cheap PIO experiments)

Try these in order, smallest diff first:

1. **Remove AUTH_IV5..15 zeroing.** May be the actual corruption trigger.
   Web/mako never touch IV5..15; only write the algorithm's IV words.
2. **Remove per-op SW_RST** (revert `c4138244a7a0`). Restores webOS pattern
   of one SW_RST at probe, never per-op.
3. **Lengthen post-AUTH_DONE IDLE wait.** Wait longer for state machine to
   reach IDLE (current 10ms may not be enough on this silicon).
4. **Try writing 0 to GOPROC after digest read.** May coax engine out of
   FINAL_READ when CRCI 15 isn't firing.

### Path A (real fix)

Match webOS exactly:

1. Rework `qce/dma.c` to chain two descriptors on one ADM channel:
   - BOX: input data → DATA_SHADOW0 (+0x8000), peripheral CRCI 4
   - SINGLE: AUTH_IV0..N (+0x100) → result_buf, peripheral CRCI 15
2. Rework `qce/sha.c` CE2 path to use this DMA chain instead of PIO.
3. Remove all per-op heuristic state mgmt.

Linux 6.18 dmaengine supports multi-descriptor chains via
`dmaengine_prep_slave_sg()` + `dmaengine_prep_slave_single()` with
descriptors submitted in order. The qcom ADM driver
(`drivers/dma/qcom/hidma.c` / `qcom_adm.c`) needs to support per-descriptor
peripheral_config CRCI selection — verify this works.

## References

- `webos-linux-kernel-touchpad/drivers/crypto/msm/qce.c` lines 540-566, 627-662, 839-952, 1090-1170, 1303-1384
- `Documents/GitHub/mako-kernel/drivers/crypto/msm/qce.c` lines 594-687
- `Documents/GitHub/mako-kernel/drivers/crypto/msm/qce40.c` lines 309-421
- `Documents/GitHub/mako-kernel/drivers/crypto/msm/qcryptohw_30.h` — CE2/3 register defs
- `Documents/GitHub/mako-kernel/drivers/crypto/msm/qcryptohw_50.h` line 493 — RESULTS_DUMP bit (CE5 only)
- `reports/ce2-investigation/htc-sbl3-ce2-pio-sequence.md`
- `reports/ce2-investigation/htc-binaries/sbl3.disasm` lines 97354-97553
