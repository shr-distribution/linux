# HP TouchPad Bluetooth (CSR BlueCore / BCSP) — investigation brief for Gemini

## Goal
Get Bluetooth working on the mainline **Linux 6.18** port for the **HP TouchPad**
(Qualcomm **APQ8060 / MSM8660**, dual ARMv7 Scorpion, LuneOS). The BT chip is a
**CSR BlueCore** (BC63B239A / "BlueCore6"; HCI manufacturer = CSR, despite the
historical "BCM4329" misnomer in our code). It speaks **BCSP** (BlueCore Serial
Protocol: SLIP-framed, channel-multiplexed, with a CRC) over a UART.

The hardware/firmware is known-good: **stock webOS drives this exact chip on the
exact same pins and it works.** We are reproducing that on mainline.

## Transport / wiring
- UART: **GSBI6**, mainline `msm_serial` driver in **UARTDM v1.3 (ADM3-era)** mode,
  exposed as `/dev/ttyMSM1`, physical base `0x16540000`.
- Pins: **gpio53=TX, 54=RX, 55=CTS, 56=RFR/RTS**, pinmux `function=gsbi6`,
  **drive strength 8 mA**, no pull. Verified live via TLMM ctl reg (`0xC4`).
- TX trace to the chip RX is a **bare 0 Ω** link (no buffer/level-shifter/mux) per
  schematic — i.e. the SoC pad drives the chip RX directly; symmetric with the
  RX direction.
- Link-establishment runs at **115200 8N1** (webOS later switches operational
  speed to 3686400; we never get past link-est).
- Power/wake GPIOs (TLMM base 512): BT_POWER=gpio130, BT_WAKE=gpio131,
  BT_RST_N=gpio138, BT_HOST_WAKE=gpio129.

## BCSP link-establishment state machine (symmetric)
Both ends send SYNC repeatedly; on RX SYNC → send SYNC_RSP; on RX SYNC_RSP →
send CONF; on RX CONF → send CONF_RSP → link up. Our driver's link-control
payloads (4 bytes each, on channel 1, CRC'd):
- SYNC      = `da dc ed ed`
- SYNC_RSP  = `ac af ef ee`
- CONF      = `ad ef ac ed`
- CONF_RSP  = `de ad d0 d0`

## CURRENT OBSERVED BEHAVIOR (on-device, mainline)
The chip **continuously transmits SYNC** every ~230 ms and **never advances**:
```
RX from chip (DMA): c0 40 41 00 7e da dc ed ed a9 7a c0   (BCSP SYNC, CRC valid)
driver: "sync received" → sends SYNC_RSP:
TX to chip:         c0 40 41 00 7e ac af ef ee bb 84 c0   (byte-identical to webOS)
chip: ... still only SYNC. Never SYNC_RSP, never CONF.
```
So the chip **ignores ALL of our TX**. Counts over a 5 s window: 23 SYNC received
and parsed, **0 SYNC_RSP received, 0 Link established**. `hci0` shows RX bytes
accumulating (chip SYNC) and TX bytes accumulating (our responses); HCI Reset
(`0x0c03`) then times out (`-110`).

Key logical point: in symmetric BCSP, if the chip had received our SYNC it would
reply SYNC_RSP; if it had received our SYNC_RSP it would send CONF. It does
**neither** → **none of our TX is reaching the chip decodably** (this is not a
content/protocol problem — a content problem would still draw *some* response).

## WHAT WE FIXED (RX is now perfect)
The mainline `msm_serial` RX DMA used a `kzalloc` buffer + per-transfer streaming
`dma_map_single(DMA_FROM_DEVICE)` / `dma_unmap_single`. On this ADM3 DMA engine,
the unmap-time **cache invalidate raced the ADM graceful-flush write retiring to
DRAM**, so the CPU read back the original zeros while the HW byte counter reported
the correct count → "correct count, **zero data**". Symptom: chip SYNC arrived as
all-zeros.
**Fix:** switch the RX buffer to **`dma_alloc_coherent`** (uncached, like the
legacy webOS hsuart), drop per-cycle map/unmap, add `dma_rmb()`. Now RX delivers
real bytes; the entire BCSP RX chain (SLIP decode, header checksum, CRC verify,
LE dispatch) works with **0 CRC failures**. Verified on both transports below.

## TX — EVERYTHING ELIMINATED (exhaustive, on-device)
Our SYNC/SYNC_RSP are **byte-identical to a captured webOS cold-handshake wire
trace** that successfully establishes the link. We have matched webOS on every
software/register-visible parameter and the chip still rejects our TX while
accepting webOS's identical bytes:

1. **Byte content / framing / CRC**: byte-identical to webOS (verified wire trace).
2. **Internal UART loopback** (set MR2 bit7 LOOP_MODE via /dev/mem): the driver
   completes the **entire** handshake against itself
   (SYNC→SYNC_RSP→CONF→CONF_RSP→**Link established**), every frame byte-perfect,
   0 CRC failures. ⇒ SoC TX **byte generation, framing, CRC, NCF/FIFO, baud are
   all correct**. *(Caveat: MR2 loopback is controller-internal, BEFORE the pad —
   so it proves the byte stream is logically perfect but does NOT exercise the
   physical pad waveform.)*
3. **TX pad**: TLMM ctl reg = `0xC4` (mux=gsbi6, drive=8 mA, no pull), identical to
   the working RX pad. A prior UART-BREAK test confirmed the pad physically
   toggles the line.
4. **Baud / parity / mode**: MR2=`0x34` (8N1); TX shares the UART clock with RX,
   and RX decodes the chip's 115200 stream perfectly ⇒ TX baud == RX baud.
5. **UART clock derivation**: matched webOS exactly (gsbi6_uart_clk=7372800 + CSR
   DIV_4, vs mainline's original fractional MND 1843200/DIV_1). **No change** —
   chip still only-SYNCs.
6. **Drive strength**: 8 mA (matches webOS gpiomux UART_ACTIVE = DRV_8MA). No change.
7. **Flow control**: CRTSCTS off (webOS FLOW_CTRL_NONE), RTS/RFR asserted (we
   drive it; chip asserts CTS back). Fully ruled out, both directions.
8. **BT_WAKE polarity** (gpio131): tried HIGH and LOW — no effect.
9. **DMA-TX vs PIO-TX**: falsified by the loopback (byte-perfect either way);
   currently PIO TX + DMA RX.
10. **serdev vs line-discipline**: **IDENTICAL** — both `hci_uart_bcsp` serdev and
    the N_HCI ldisc (hciattach/btup) receive the chip SYNC perfectly over DMA and
    both fail to make the chip respond (0 SYNC_RSP). An earlier one-off "ldisc
    established the link" was **not reproducible** and is now considered a fluke.

## webOS reference (the thing that works)
- webOS uses a **userspace BCSP stack** (`PmBtStack -F 26000 -U 3686400`) over a
  **custom `hsuart` driver** (`/dev/ttyHS0`), same GSBI6 pins.
- Captured webOS cold handshake: webOS TX SYNC → chip SYNC (~274 ms) → webOS
  SYNC_RSP → **chip CONF** → CONF_RSP → link up. So the chip acts on webOS's
  SYNC_RSP but never on ours, with **byte-identical** content.
- The one observable difference in the webOS bring-up: immediately before TX SYNC,
  webOS does a **pin-mux toggle + RTS-as-GPIO dance**:
  `btuart_pin_mux ON → deassert_rts(get/put) → pin_mux OFF → pin_mux ON →
  deassert_rts ...0(get)`. mainline `msm_serial` does not do this. A crude
  "pin-mux glitch" reproduction (gsbi6→GPIO→gsbi6) was tried once and did **not**
  help, but may not have replicated the exact sequence/timing.

## CURRENT CONCLUSION
RX is solved. TX byte-generation is provably perfect (loopback). Every
software/register/clock/drive/flow/transport parameter matches webOS. The chip
accepts webOS's electrically-driven bytes on the same pins but not ours.
⇒ The remaining difference is the **physical TX waveform / sub-bit timing on
gpio53** (mainline `msm_serial` UARTDM vs webOS `hsuart`), which is invisible to
software and to the internal loopback. The only clearly-remaining diagnostic is a
**scope / logic analyzer on gpio53** comparing mainline-TX vs webOS-TX.

## QUESTIONS FOR GEMINI
1. What physical-layer differences could make a **CSR BlueCore** UART RX reject a
   byte-perfect, correctly-framed, correctly-CRC'd BCSP SYNC_RSP at **115200 8N1**,
   when (a) the chip's own TX on the symmetric 0 Ω trace is received fine, and
   (b) a *different* UART driver (webOS hsuart) sending **byte-identical** data on
   the **same pins** works? We've ruled out content, baud, parity, drive, flow,
   wake, DMA-vs-PIO, serdev-vs-ldisc.
2. **Fractional-N clock jitter hypothesis**: mainline derives 115200 via a
   fractional MND divider; webOS integer-divides a 7.3728 MHz source (CSR /4).
   We tried matching that and it didn't fix it — but could residual UART bit-clock
   jitter still cause the CSR RX UART to mis-sample our bytes (while *we* sample
   the chip's cleaner clock fine, since UART sampling tolerance is asymmetric)?
   How would you confirm/measure this short of a scope (e.g., any MSM UARTDM
   register that reveals actual baud/oversampling, or a way to force a known-clean
   integer divider)?
3. **The webOS pin-mux/RTS dance before TX**: is there a known CSR BlueCore
   requirement for a specific **line condition before it will listen** — a BREAK,
   an idle-high settle time, a glitch that triggers autobaud, or an RTS/CTS edge
   sequence? If so, what exactly, and how to replicate it from `msm_serial`?
4. Could the chip boot with an **asymmetric baud** (RX expecting a different rate
   or an autobaud training pattern than its TX), such that our 115200 SYNC_RSP is
   undecodable to its RX even though its TX is 115200?
5. Are there CSR-specific BCSP timing requirements (SYNC cadence, a tight response
   window for SYNC_RSP relative to the chip's SYNC) that a userspace stack
   (webOS PmBtStack) would satisfy but a kernel ldisc/serdev driver might miss?
6. Any non-scope on-device experiments left that could discriminate
   "pad waveform" vs "chip-side state" as the failure?

## SECONDARY ISSUE (unrelated to BT — also want input)
The device suffers **hard hangs from kernel memory corruption** whenever the
**VIDC H.264 video ENCODER** is streaming. Two captured Oopses, both with
`PC at <ext4 function>+0x0` (e.g. `ext4_getattr+0x0`, `mpage_process_page_bufs+0x0`)
— i.e. a faulting **write to kernel memory** / corrupted function entry — both
occurring in the same millisecond as `qcom-vidc` encoder activity (codec=16,
recon buffers programmed at SMI physical `0x38600000`, SEQ_HEADER done,
start_streaming). Signature is consistent with a **stray DMA write from the VIDC
encoder scribbling over kernel DRAM**. BT modules are merely "linked in" and are
not the cause. Question: what in a Qualcomm VIDC (vdec/venc, MSM8660) encoder
recon-buffer or output-buffer DMA setup most commonly causes out-of-bounds DMA
into kernel memory (e.g. recon/DPB sizing, an output buffer physical address not
in the SMI window, missing IOMMU/no-IOMMU direct-physical addressing)?
```
[147s] qcom-vidc: recon pool: 4 slots × (y=98304 c=49152), total 589824 @ 0x38600000
[147s] qcom-vidc: encoder SEQ_HEADER done ... start_streaming
[151s] Unable to handle kernel paging request at 8198b240 when write
       Internal error: Oops: 80d   PC is at ext4_getattr+0x0/0x1a8
```
