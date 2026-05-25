# HP TouchPad BT (CSR BC63B239A / BCSP) — update for Gemini, 2026-05-25

Follow-up to your 6-point analysis (RX power-gating, fractional jitter, the
webOS pin-mux/RTS "dance", baud sweep, break-reset, turnaround timing). We
acted on all of it. **Net: every digital / register / clock / voltage
parameter is now proven identical between webOS and mainline, yet the chip
still decodes webOS's TX and never ours. We have no oscilloscope.** Asking
whether any software-measurable angle remains.

## Recap of the wall (unchanged)
RX is solved (DMA coherent buffer). On TX the chip streams BCSP SYNC
(`c0 40 41 00 7e da dc ed ed a9 7a c0`) forever and never acts on our
byte-perfect SYNC_RSP. Internal UART loopback completes the *entire*
handshake against ourselves (bytes/CRC provably correct). Same gpio53 pins;
schematic shows a bare-0Ω TX trace (no buffer/level-shifter/mux).

## Your architecture question (how to time the glitch right before TX)
We answered it from the **webOS source**, not guesswork. The board callbacks
`btuart_pin_mux` + `btuart_deassert_rts` are invoked by the legacy `hsuart`
driver as a full **UART port disable → re-init immediately before the first
SYNC**: `__msm_uartdm_port_disable()` un-muxes pins; `__msm_uartdm_port_init()`
re-muxes + sets baud + **RESETS the UART**; then `__set_rx_flow()` re-asserts
RTS. So the "dance" is really *mux-glitch + UART reset + RTS re-assert*, and
our earlier pin-mux-only glitch missed the **reset**.
Key constraint we found: mainline `msm_set_termios` (which calls the UART
reset) runs under the atomic `uart_port_lock`, so pinctrl (which sleeps)
can't go there. But `hci_uart`'s write path is a **workqueue (sleepable)**,
so we do the reset from the bcsp driver's open/TX context via
`serdev_device_set_baudrate(115200)` → `msm_set_termios` → `msm_reset`.

## What we tried since (each → result)
1. **Pin-mux glitch** (msm_serial `qcom,startup-mux-glitch` + DTS gpio state,
   commits 27416b8fb03a + 617f9bf63c78): ran (`startup pin-mux glitch
   applied`), **chip still only-SYNCs**. Also fired ~140 ms before TX (vs
   webOS's ~725 µs) and missed the reset.
2. **UART reset-before-SYNC + RTS re-assert** (bcsp_open, commit 7531750a332f):
   ran, **chip still only-SYNCs** (0 sync_rsp). NOTE: `msm_reset()` clears
   `MR1 RX_RDY_CTL`, so it deasserts RTS — we re-assert after, matching webOS
   order. No effect.
3. **0xFF TX preamble** (module param, swept 4/8/16 bytes before the SLIP
   `c0`, your idea #6.1): **no effect**.
4. **Baud sweep** (termios2/BOTHER, 112000–118000): `uart_clk` verified
   *changing* per request (7372800 ↔ 14745600), so real baud varied — **chip
   only-SYNCs at every rate**. At nominal 115200 we run **clk=7372800 = clean
   integer (7372800/64 = exactly 115200, 0% error)** and still fail. So
   baud-offset / clock-derivation is **not** the cause; the datasheet's ≤1%
   RX tolerance is met.
5. **Long break-to-reset** (hold TX low 1.3 s, > `PSKEY_HOSTIO_UART_RESET_TIMEOUT`):
   chip re-inits and resumes SYNC, **still only-SYNCs**.
6. **serdev vs ldisc vs raw-userspace**: all three tested, **identical**
   (chip only-SYNCs). Transport is not the differentiator. (webOS itself used
   a *userspace* stack `PmBtStack`, closest to our ldisc path, not serdev.)

## New hard data we gathered (scope-free)
- **Datasheet (BC63B239A BlueCore6):** transport is **pin-strapped** at boot
  (Table 9.1: SDIO_CLK/CMD = 00 bcsp / 01 h4 / 10 h4ds / 11 h5); UART RX baud
  tolerance **≤1%** (Table 9.2); a long break on RX **resets the chip** (9.2).
- **Schematic:** SDIO_CLK/CMD (the transport strap) go only to a test point
  TP2900, tied low (=bcsp), **no SoC GPIO** → **H4 is not software-selectable**
  (would need rework), and H4 wouldn't help anyway (same pins/waveform).
- **Clock synthesis comparison (THE key new datum)** — read the GSBI6 UART
  GCC M/N/D registers on **webOS (via /dev/mem)** vs **mainline**:
  - `UART_APPS_MD (0x00902A70)` = `0x0060FD8E` on **BOTH** (M = 96).
  - `UART_APPS_NS (0x00902A74)` = webOS `0xFDEE0B43` vs mainline `0xFDEE0143`
    — differ **only** by `0x0A00` = `BIT(11) root_en | BIT(9) br_en`, i.e.
    the clock **enable** bits (webOS clock ON / mainline gated at read). The
    **synthesis** bits (N=`0xFDEE`, src/predivide/MNCNTR-mode=`0x43`, MNCNTR_EN
    bit8) are **identical**. Freq table: `gsbi_uart = PLL8 × M/625`.
  - ⇒ **The fractional-divider setup is identical** → "different MND jitter"
    (your #2) is **falsified at the register level**. Both synthesize the UART
    clock the same way off the same PLL8.
- **Rail voltages (webOS):** BT supply `pm8058_s3 = 1.8 V`, always-on; **no
  regulator changes when BT toggles** (GPIO-gated). UART I/O is 1.8 V both
  ends; path is symmetric and the chip's own TX reaches us fine → signal
  *level* is not the asymmetry.
- **Pad config:** gpio53–56 TLMM ctl = `0xC4` (gsbi6 mux, 8 mA, no pull) on
  both; identical to the working RX pad.

## Current conclusion
Everything software/register/voltage-visible is now **proven identical** to
webOS: bytes, CRC, **clock synthesis (M/N/D/src)**, baud (clean 7372800),
parity/mode, flow/RTS, drive strength, pad mux, rail voltages, transport. The
chip accepts webOS's electrically-driven bytes on the same 0 Ω-jumper pins
and rejects ours. By elimination the difference is the **analog TX waveform
on gpio53** — edge slew / rise-fall / cycle-to-cycle jitter / inter-byte gap
— which no register or `/sys` value can report. The only remaining UART
driver difference is `msm_serial` (UARTDM) vs webOS `hsuart`, but a prior
analysis showed they emit byte-identical NCF/FIFO output.

## The question for you
**We have no oscilloscope/logic analyzer.** Given that:
1. Is there ANY remaining software-readable signal that could distinguish the
   two TX waveforms — e.g. a hidden TLMM pad register (slew-rate / keeper /
   HDRV beyond the ctl reg), a GSBI/TCSR config bit, a UART_DM TX-FIFO/NCF
   timing knob, or a way to use the chip's RX (or PMIC ADC) as a crude
   detector — that we haven't checked?
2. Could `msm_serial`'s UARTDM TX engine produce **inter-byte gaps** that
   `hsuart` doesn't, even with identical bytes/baud/clock (e.g. NCF
   re-arming, FIFO watermark TFWR, DMA vs PIO TX feeding) — and is there a
   software way to detect/measure that without a scope?
3. Is there a known CSR BlueCore quirk where the RX UART needs something we
   can't see in registers (a specific idle-line duration, a wake-on-edge, a
   parity/stop-bit subtlety) that webOS's hsuart happens to satisfy?
4. Failing all that — do you agree this is now genuinely scope-only, and is
   there a cheaper proxy than a scope (e.g. a second UART/MCU we could wire to
   gpio53 to capture the byte stream, an Arduino/FTDI-as-analyzer trick)?

(Commits: glitch 27416b8fb03a+617f9bf63c78; reset+preamble 7531750a332f;
btup baud/break 64cb55e7d189; BT disabled-at-boot 772e3710d0e8.)
