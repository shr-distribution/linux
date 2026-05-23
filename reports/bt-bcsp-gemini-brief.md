# HP TouchPad Bluetooth (CSR BlueCore over GSBI6 UART) — request for fresh input

## TL;DR / the question

On the HP TouchPad (Qualcomm APQ8060 / MSM8660), a **CSR BlueCore6 (BC63B239A)**
Bluetooth chip talks to the SoC over a UART using **BCSP** (BlueCore Serial
Protocol). The chip works perfectly under the original webOS 2.6.35 kernel
(driven by Qualcomm's `hsuart` high-speed-UART driver + a userspace BCSP stack).

On mainline Linux 6.18 (driven by the standard `msm_serial` / UARTDM driver +
the kernel `hci_bcsp` line discipline), **the chip continuously transmits BCSP
SYNC and never responds to anything we send back.** It is stuck in BCSP
link-establishment state forever.

We have matched webOS on *every* software-visible parameter we can find — exact
on-wire bytes, baud, parity, clock source, clock divider, flow-control, pad
drive strength, GPIO states — and the chip still ignores our TX while accepting
webOS's byte-identical TX on the same physical pins.

**We believe the remaining difference is the physical TX waveform produced by
`msm_serial`/UARTDM vs `hsuart`. We want a sanity check: is that conclusion
sound, and is there any software/register avenue we have not considered?**

---

## Hardware

- SoC: Qualcomm APQ8060 (MSM8660), dual Scorpion ARMv7.
- BT chip: **CSR BC63B239A, BlueCore6-ROM, WLCSP 51-pin** (confirmed from board
  schematic). HCI manufacturer reads as Cambridge Silicon Radio. NOT Broadcom
  despite legacy "BCM4329" naming in old code.
- Transport: UART on **GSBI6** (Qualcomm General Serial Bus Interface block #6).
  - Base 0x16540000 (UARTDM core), GSBI wrapper 0x16500000.
  - APQ8060 pads: gpio53 = BT_TX, gpio54 = BT_RX, gpio55 = BT_CTS, gpio56 = BT_RTS.
- Schematic-verified UART net topology (SoC pad → series R → CSR pin):
  - gpio53 TX  → **0Ω** → CSR UART_RX
  - gpio54 RX  ← **0Ω** ← CSR UART_TX
  - gpio55 CTS ← **0Ω** ← CSR UART_RTS
  - gpio56 RTS → **0Ω** → CSR UART_CTS
  - **The TX path is a bare 0Ω jumper** — no buffer, no level shifter, no analog
    mux, no RC filter. Both ends are 1.8 V. So there is no board-level component
    that could shape/asymmetrize our TX vs webOS's.
- BT control GPIOs: BT_POWER=130, BT_WAKE=131 (chip PIO[1], host→chip wake),
  BT_HOST_WAKE=129 (chip→host), BT_RST_N=138 (active low).

## BCSP protocol facts

BCSP link establishment is a 4-way handshake using fixed packets:
- SYNC     payload `da dc ed ed`
- SYNC-RSP payload `ac af ef ee`
- CONF     payload `ad ef ac ed`
- CONF-RSP payload `de ad d0 d0`

The chip, after power-up/reset, repeatedly sends SYNC and waits for SYNC-RSP.
On receiving SYNC-RSP it should send CONF; the handshake then completes and the
link goes "up". Our chip **never leaves the SYNC stage**.

On-wire SLIP-framed packets we transmit (chan 1, with CRC, byte-identical to
what webOS sends — captured from webOS hsuart trace):
```
SYNC     c0 40 41 00 7e da dc ed ed a9 7a c0
SYNC-RSP c0 40 41 00 7e ac af ef ee bb 84 c0
CONF     c0 40 41 00 7e ad ef ac ed a1 a6 c0
CONF-RSP c0 40 41 00 7e de ad d0 d0 83 58 c0
```
(0xC0 = SLIP delimiter; byte1 0x40 = the BCSP header's reliable/CRC bit; trailing
2 bytes before the closing 0xC0 are the 16-bit CCITT CRC.)

## The core symptom (reproduced on-device today)

Running a freestanding userspace BCSP driver (`btbcsp`, bypasses the kernel
hci_bcsp/ldisc entirely — opens the tty, drives the handshake by hand) on the
mainline kernel:

```
BT_WAKE gpio643 asserted HIGH                 (gpio base 512 + 131)
opened /dev/ttyMSM1, modem=0x166, raw 8N1 115200, RTS asserted
[ 122] RX(12): c0 40 41 00 7e da dc ed ed a9 7a c0  -> got SYNC, sent SYNC-RSP
[ 364] RX(12): c0 40 41 00 7e da dc ed ed a9 7a c0  -> got SYNC, sent SYNC-RSP
[ 607] RX(12): c0 40 41 00 7e da dc ed ed a9 7a c0  -> got SYNC, sent SYNC-RSP
... repeats forever; chip never sends CONF, never acknowledges our SYNC-RSP
```

Key reads at the moment of failure:
- We **receive the chip's SYNC perfectly** (clean 12-byte SLIP frames, correct
  CRC) → our RX path and baud lock are correct.
- `modem=0x166` → CTS bit (0x020) is set, i.e. the chip's RTS (our CTS input) is
  asserted: the chip says it is ready to receive from us.
- We assert RTS (the chip's CTS) → we are telling the chip it may send.
- We TX SYNC-RSP every 250 ms. The chip never reacts.

So: chip is powered, awake, flow-control-ready, transmitting, and we decode its
TX flawlessly — but our TX has zero effect on it.

## webOS reference (the thing that WORKS)

Same chip, same board, same pins, original webOS 2.6.35 kernel:
- UART driver: Qualcomm `hsuart` (high-speed UART, MSM8660-specific), NOT the
  generic `msm_serial`.
- Userspace BCSP stack (`PmBtStack`) drives the same handshake over `/dev/bt_uart`.
- Link establishes; full HCI traffic flows; BT is fully functional.
- Captured webOS register/clock config at the 115200 link-establishment phase
  (via a debug build of the webOS kernel with logging added):
  - UART clock = **7372800 Hz**, CSR (clock-select) divider = **DIV_4**
    → 7372800 / 4 / 16 = 115200 baud.
  - MR1 = 0x34, MR2 = 0x34 (8 data bits, no parity, 1 stop bit; HW flow gating off).
  - GSBI6 protocol code = UART (GSBI_CTRL = 0x40).
  - (After link-est, webOS reconfigures to clk=58982400 / DIV_1 / 3686864 baud for
    operation, but link establishment itself is at 115200.)

## What we have changed/matched and FALSIFIED on the mainline side

Every item below was tested on real hardware and did **not** fix it (chip still
only-SYNCs):

1. **Exact on-wire bytes** — verified our SYNC/SYNC-RSP are byte-identical to
   webOS's, including the CRC and the BCSP header CRC/reliable bit.
2. **CRC on the link-establishment channel** — found and fixed a real bug where
   the driver stripped the CRC from BCSP channel 1; now byte-perfect. (Necessary
   correctness fix, but not sufficient.)
3. **Baud / clock source / divider** — patched `msm_serial` so the BT UART at
   115200 uses clk=**7372800** with **CSR DIV_4**, exactly like webOS (instead of
   mainline's default 1843200 / DIV_1 path). Verified `clk_get_rate`=7372800 on
   device. No change.
4. **Parity / framing** — MR2=0x34 (8N1) confirmed; also tried even-parity (0x36).
5. **Hardware flow control** — tried with and without CRTSCTS; MR1 CTS/RX_RDY
   gating bits off (MR1=0x34). Confirmed our TX is not being gated by CTS.
6. **RTS assertion** — explicitly assert RTS (chip's CTS) before/throughout. CTS
   from chip confirmed asserted (modem 0x166).
7. **Pad drive strength** — set gpio53-56 to 8 mA to match webOS gpiomux (was
   default). No change.
8. **Reset sequence + settle timing** — assert BT_RST_N (active-low) 100 ms,
   release, 1 s settle, matching webOS-ish timing.
9. **BT_WAKE / deep-sleep** — TODAY's test: asserted BT_WAKE (chip PIO[1]) high
   and confirmed gpio=1 during the handshake. CSR chips can gate their UART RX in
   deep sleep while still TXing SYNC, so this looked promising — but it made no
   difference. **Falsified.**
10. **CSR PSKEY init** — the legacy driver had scrambled PSKEY #define IDs that
    would write garbage; we skip PSKEY writes entirely (correct hygiene). Chip
    still SYNCs continuously with or without, and survives 30 s power-off sending
    clean data, so the crystal/PSKEYs are not bricked.
11. **GSBI protocol mode** — GSBI_CTRL=0x40 (UART) confirmed on device.
12. **skip-sync / starting state** — the chip sends SYNC after every power-up and
    every warm reset, so we must complete the handshake (cannot skip it).

The only structural difference left between the working and non-working cases is:
**webOS uses the `hsuart` driver; mainline uses `msm_serial`/UARTDM.** Both target
the same UARTDM hardware block, but their TX FIFO handling / DM-mode framing /
timing differ.

## Our current hypothesis

The chip's UART receiver rejects our TX at the **physical/electrical or
bit-timing level** — something `msm_serial`/UARTDM does to the actual waveform on
gpio53 that `hsuart` does not (candidate causes: inter-character gaps from
UARTDM's "DM mode" where you program NO_CHARS_FOR_TX then stream through the TX
FIFO; a stop-bit / line-idle glitch; a TX-enable transient; a subtly wrong actual
bit rate despite the clock matching). The chip tolerates webOS's waveform and not
ours. Since the board has zero active components on the TX path, this can only be
the SoC pad output itself.

Planned next step: logic analyzer / oscilloscope on gpio53, capturing a mainline
SYNC-RSP vs a webOS SYNC-RSP byte-for-byte to find the waveform/timing delta.

## Questions for Gemini

1. Is the "physical TX waveform" conclusion sound given the falsification list,
   or is there a software/register avenue we have overlooked?
2. MSM8660 UARTDM specifics: are there known issues with **DM mode TX framing**
   (NO_CHARS_FOR_TX / TFWR / TX FIFO) that introduce inter-byte gaps or a
   start/stop-bit anomaly a peer UART could reject? Does `hsuart` use a different
   TX path (e.g. DMA vs PIO, or "single-character" vs "DM" mode) that would
   produce a cleaner stream?
3. Could a CSR BlueCore reject a SLIP/BCSP frame purely due to **inter-character
   timing** (gaps between bytes within a frame), even if every byte and the CRC
   is correct? BCSP/SLIP is byte-framed (0xC0 delimiters), so in principle it
   should be gap-tolerant — but is there a CSR-specific receive timeout?
4. Are there UARTDM registers beyond MR1/MR2/CSR/IPR that affect the **output**
   bit timing or stop-bit length (e.g. RFWR/TFWR thresholds, IPR stale-timeout,
   the bit-rate fine-tuning) that we should match to webOS exactly?
5. Anything about the **actual achieved baud** — UARTDM derives baud from
   clk / CSR / 16; clk=7372800, DIV_4 → exactly 115200. Is there a known UARTDM
   quirk where the realized rate deviates from the ideal, and could `hsuart` use a
   different (non-integer-divider) clock that lands closer to the chip's expected
   rate?

## Appendix: tools / artifacts (in repo)

- `reports/bt-trace/btbcsp.c` — userspace BCSP handshake driver (bypasses kernel),
  now auto-asserts BT_WAKE.
- `reports/bt-trace/btup.c` — minimal hciattach/hciconfig-up replacement (kernel
  hci_bcsp path), also auto-asserts BT_WAKE.
- `reports/bt-trace/webos-hsuart-trace-2026-05-22.log` — full webOS TX/RX byte
  trace of a working handshake (the reference).
- `reports/bt-trace/webos-cold-handshake-2026-05-22.log` — webOS cold-boot
  handshake.
- `reports/bcm4329-bluetooth-analysis.md` — full corrected analysis.
- Mainline driver: `drivers/bluetooth/hci_bcsp.c`; UART driver:
  `drivers/tty/serial/msm_serial.c` (BT-UART clock patch).
