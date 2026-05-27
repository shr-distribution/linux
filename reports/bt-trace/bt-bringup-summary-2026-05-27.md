# HP TouchPad Bluetooth Bring-up — Consolidated Summary

> Status as of 2026-05-27. Consolidates the legacy-binary deep dives, the on-device
> falsification log, and the byte-level match analysis. Companion to
> `dead-ends.md`, `refs/bt-chip-datasheet.md`, `refs/hp-uart-tweaks.md`,
> `refs/csr-bcsp-research.md`, `refs/legacy-bt-audit.md`.

---

## 1. The chip (settled, definitive)

- **CSR BlueCore6-ROM, BC63B239A** (BT 2.1+EDR). NOT Broadcom — the pervasive
  "BCM4329 / palm,bcm4329-bcsp" naming is a misnomer carried over from the WiFi
  combo (WiFi is a *separate* Atheros AR6003 / WCN1314).
- Confirmed by: schematic, HCI manufacturer ID = 10 (Cambridge Silicon Radio),
  BCSP transport (CSR-proprietary), and the webOS binary string
  **"CSR-SYNERGY-BC6 ROM"** in `libPmBtBsaif.so`.
- **ROM chip → no patchram/.hcd firmware.** Configured at runtime via PSKEY/BCCMD
  over BCSP. (Do NOT look for a `.hcd`; there isn't one.)

## 2. The wiring (settled)

- BT UART = **GSBI6 / UART1DM** on MSM8660/APQ8060.
  - gpio53 = TX, gpio54 = RX, gpio55 = CTS, gpio56 = RTS/RFR. Pad func = FUNC_1, 8 mA, no pull.
- Initial + operating baud = **115200, 8N1** (ROM default; chip SYNCs cleanly at
  115200, so its BCSP-LE baud is fixed there).
- Crystal = **26 MHz** (PSKEY_ANA_FREQ), Ftrim 0x19.
- Control GPIOs (WiFi board): BT_RST_N=138 (active low), BT_POWER=130 (high=on),
  BT_WAKE=131, BT_HOST_WAKE=129 (IRQ).
- Legacy webOS (2.6.35 AND 3.0.5) ran the **entire BCSP+HCI stack in USERSPACE**
  over a raw `/dev/bt_uart` (custom `hsuart` misc char dev, PIO TX + DMA RX);
  `CONFIG_BT` was **not set**. There is NO HP reference for the in-kernel
  `hci_bcsp` path that mainline uses.

## 3. The blocker (the one unsolved problem)

On mainline 6.18 (`msm_serial` UARTDM + in-kernel `hci_bcsp`), the chip
**emits BCSP SYNC forever (~every 250 ms) and never accepts any host TX** — it
never sends a SYNC-RSP to our frames, so link establishment never advances past
`BCSP_LINK_UNINIT`. RX (chip→host) works perfectly; TX (host→chip) is ignored at
the chip. The userspace bypass (`reports/bt-trace/btbcsp.c`) fails identically.

---

## 4. What MATCHES webOS (verified reproduced) ✅

| Aspect | webOS | mainline now | Evidence |
|---|---|---|---|
| Chip ID / transport | CSR BC6-ROM, BCSP | BCSP via `hci_bcsp` | manuf=10, datasheet ref |
| Baud | 115200 8N1 | 115200 8N1 | chip SYNC decodes clean |
| UART clock | NS=0xfd9a0b43 MD=0x000cfd8e (7372800/DIV_4, PLL8) | byte-identical | commit c589b28172f7; webOS TPADUART dump |
| Crystal/Ftrim | 26 MHz / 0x19 | (chip-internal, matched) | libPmBtBsaif `.data` |
| HOST_INTERFACE | 0x0001 = BCSP | BCSP | PSKEY 0x01F9 dump |
| SYNC-RSP bytes | `c0 40 41 00 7e ac af ef ee bb 84 c0` | byte-identical | `bcattach` trace vs TXWIRE |
| **BCSP CRC** | LE frames CRC'd (hdr 0x40 + trailing CRC) | `use_crc=1` set at open, applied to all channels incl. LE | hci_bcsp.c:653-670, 2655 |
| Flow control | NONE for TX (CTS not gating); RFR/RTS asserted | CTS_CTL clear; `TIOCM_RTS` asserted for LE | hci_bcsp.c:2630-2643 |
| TX waveform at FIFO | one-shot, gap-free | BTTX trace: byte-perfect, gap-free | on-device trace |
| Full UART reg set | MR1=0x34 MR2=0x34 IPR=0xa | every TX-relevant reg matches | webos-mt9m113-full-dmesg.txt:612 |

## 5. What we TRIED and FALSIFIED ❌ (do not re-run)

1. **H2 — UART-RX power-gate wake dance.** Synchronous RTS pulse + TX/RFR pin-mux
   glitch before each TX (commit cac396b85304). Chip still 0 SYNC-RSP.
2. **Forced cold power-cycle** (500 ms, commit fb25af7b1e63) before link-est. No change.
3. **H3 — fast retransmit "hammer"** (commit 5520f9547f39): ubcsp retransmits LE
   frames every ~2.5 ms (~40× the chip's 250 ms SYNC). Tested at burst 6/9/20; at
   20 = continuous 513 KB TX in 2 s. Chip STILL 0 SYNC-RSP — decodes none of our
   frames at any cadence.
4. **Clock jitter** — NS/MD already jitter-matched to webOS exactly. Ruled out.
5. **Queue-purge race** — EXONERATED (userspace btbcsp.c fails identically).
6. **No-CRC LE frames** — was an *earlier* bug; fixed. CRC now on for LE (see §4).
   This was a genuine bug we caught, but fixing it did not unblock TX.
7. **Coherent RX DMA buffer** (commit 641d887590d9) — fixed a "correct count, zero
   data" RX-DMA bug; RX now byte-perfect. (Not the TX blocker, but a real fix.)
8. **Binary deep dive (2026-05-27)** — decompiled `libPmBtBsaif.so` /`PmBtEngine`/
   `libPmBtOs.so`. Found only config/bootstrap/app layers running OVER an
   established link. Extracted exact transport config (`uartConfigBcsp=0x082e`,
   host-wake params); the only meaningful unmatched bit (CRC) was already handled.
   No new pre-link software lever exists.

## 6. What does NOT match / remains UNKNOWN ⚠️

- **The physical TX waveform on gpio53.** This is the only thing never directly
  compared between webOS (`hsuart`, PIO TX) and mainline (`msm_serial` UARTDM, PIO
  TX). Everything *digital* (bytes, framing, CRC, baud divisor, clock, flow lines)
  matches; the analog edge timing / line levels on the pin have never been scoped.
  Suspected differences could be: start-bit edge timing, idle-line level/polarity,
  inter-byte spacing at the pad (vs at the FIFO), or a subtle TX driver-strength /
  rise-time issue specific to the UARTDM pad vs the legacy hsuart path.
- `uartConfigBcsp = 0x082e` is extracted but only partially decoded (CRC bit
  confirmed/handled; full PSKEY_UART_CONFIG_BCSP bitmap not in our refs). The
  remaining bits are almost certainly framing (stop/parity), which are empirically
  excluded by the clean chip→host SYNC decode.

## 7. Verdict & next step

**All software levers are exhausted.** Byte-for-byte, mainline reproduces what
webOS sends. The remaining gap is physical-layer and is **blocked on
instrumentation**: it needs a scope or logic analyzer on **gpio53** to capture and
compare webOS's vs mainline's SYNC-RSP edges on the same pin. No instrument →
no further progress on the TX blocker.

All committed changes are benign (safe defaults), pushed to
`shr-github tenderloin/6.18/upstream-patches`. Keep `CONFIG_BT=m` (=y regresses
WiFi/eMMC). Do NOT set `skip_pskeys=false` before fixing the scrambled PSKEY id
map (it bricks the chip).
