# HTC sbl3 Bootloader: Exact CE2 PIO Sequence

Date: 2026-05-16
Source: HTC PG86IMG.zip → sbl3.img (third-stage bootloader, MSM8960)
Disassembly: reports/ce2-investigation/htc-binaries/sbl3.disasm

## The Sequence (from sbl3.disasm)

HTC's sbl3 bootloader drives CE2 entirely via **CPU PIO**. No ADM DMA, no DATA_SHADOW0, no CRCI handshake. Two functions implement it.

### Function A — config + trigger (sbl3 @ 0x5f244)

CE2 base in r6 = 0x18500000.

```
str r5, [r6, #0x30]   ; SEG_CFG = algorithm/mode bits
str r0, [r6, #0x34]   ; ENCR_SEG_CFG = size << 16  (cipher) -- AUTH_SEG_CFG for hash
str r0, [r6, #0x3C]   ; SEG_SIZE = total bytes

; if cipher: write key/iv
str r0, [r6, #0x70]   ; CNTR0_IV0
str r0, [r6, #0x74]
str r0, [r6, #0x78]
str r0, [r6, #0x7C]
mvn r0, #0
str r0, [r6, #0x80]   ; CNTR_MASK = 0xFFFFFFFF

; if cipher with AES round keys: write 60 words
loop:
  str r0, [0x18500200 + r5<<2]   ; AES_RNDKEY0..59 at +0x200..+0x2EC

; trigger
mov r7, #1
str r7, [r6, #0x40]   ; GOPROC = 1     <-- BEFORE any data writes

; wait for engine to accept the GO
poll:
  ldr r0, [r6, #0x20]
  tst r0, #0x1C00     ; CRYPTO_STATE bits 12:10 (any non-zero state)
  beq poll            ; loop while still IDLE
```

**Key:** GOPROC is fired *before* feeding data. CE2 then waits in PROCESSING for DIN_RDY-gated writes.

### Function B — feed data + wait done (sbl3 @ 0x5f128)

```
; for each 4-byte chunk:
wait_din:
  ldr r0, [r7, #0x20]   ; STATUS
  tst r0, #4            ; bit 2 = DIN_RDY
  beq wait_din          ; wait until input FIFO can accept

str data, [r7, #0x00]   ; DATA_IN  <-- POST-OFFSET 0, not DATA_SHADOW

; ... repeat for all input words ...

; wait for completion
wait_done:
  ldr r0, [r7, #0x20]
  tst r0, #2            ; bit 1 = AUTH_DONE
  beq wait_done

; check error
  ldr r0, [r7, #0x20]
  tst r0, #1            ; bit 0 = SW_ERR
  ; non-zero r9 means error
```

## Reconciling with previous analyses

| Document | Claim | Verdict |
|---|---|---|
| webOS qce.c | Data goes to DATA_SHADOW0 (0x8000), result from AUTH_IV0 (0x100) | True for **ADM box descriptors**, not for PIO. SHADOW is the ADM-side bus target. |
| Earlier `qce-complete-init-sequence.md` | Poll bit 3 then bit 4 of STATUS | Misidentified — those are DOUT_RDY/ERR_INTR, not AUTH_DONE |
| Gemini ADM-domain theory | Channels locked by TZ | Falsified — diagnostics showed domain=0 |
| Our PIO test | Wrote DATA_SHADOW0 → CRYPTO_STATE stuck PROCESSING | The write went to the wrong address; CE2 only consumes DATA_IN under DIN_RDY gating |

## What our driver needs to change

1. **PIO test**: revert DATA_SHADOW0 → DATA_IN, do GOPROC before writes, poll DIN_RDY before each 4-byte write.
2. **DMA TX channel for hash**: we have *no* working DMA-mode CE2 hash reference. Bootloaders use PIO. The webOS Linux qce.c does use ADM with shadow registers, but our Linux 6.18 dmaengine model (separate rx/tx channels) doesn't easily map to it.
3. **Workaround path for hash**: keep RX DMA for input data → DATA_SHADOW0 with CRCI 4, but on completion read AUTH_IV0..N via CPU PIO instead of TX DMA. This dodges the CRCI 15 question entirely.

## Open questions

- Does CE2 accept PIO writes to DATA_IN regardless of whether SEG_CFG has SEG_SIZE > 0?
- Is there an internal padding that handles partial final blocks for SHA, or must we pre-pad to block size?
- What happens if we GOPROC before configuring SEG_CFG — does it reject?
