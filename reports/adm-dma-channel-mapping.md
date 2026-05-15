# ADM (Application Data Mover) DMA — Full Channel Mapping on MSM8660 / APQ8060 (Tenderloin)

This is the ground-truth reference for the on-SoC ADM DMA controllers, captured by reading register state from a fully booted **webOS 2.6.35-palm-tenderloin** system (the original factory OS, which uses the legacy `drivers/dma/msm_dmov.c` driver and represents the validated hardware configuration).

It exists because mainline `drivers/dma/qcom/qcom_adm.c` has several Tenderloin-specific quirks that aren't obvious from the upstream binding, and bringing up new peripherals on ADM (QCE crypto, NAND, audio, etc.) requires knowing the right CRCI values, burst encodings, and EE-window to write to. Without this reference, every new bring-up rediscovers the same gotchas the hard way (we wedged eMMC once doing exactly that — see "EE window quirk" below).

---

## 1. ADM Architecture Recap

Two ADM controllers on MSM8660:

| Instance | MMIO base | DT node | Channels | Used for |
|---|---|---|---|---|
| **ADM0** | `0x18320000` | `adm_dma0` | 0–15 | QCE crypto, TSIF, GSBI10/HSUART2, GP, NAND_CMD |
| **ADM1** | `0x18420000` | `adm_dma1` | 0–15 | NAND, all SDCC channels, HSUART1 |

In legacy `arch/arm/mach-msm/include/mach/dma.h` channel numbers are continuous **0–31** across both instances:
- Legacy chan **0–15** = ADM0 chan 0–15
- Legacy chan **16–31** = ADM1 chan 0–15
- (e.g. `DMOV_SDC1_CHAN = 18` → ADM1 chan 2)

CRCI numbers are **per-ADM**, not global: ADM0 CRCI 4 (= CE_IN) and ADM1 CRCI 4 (= SDC2) are different physical lines.

---

## 2. The EE Window Quirk (THE thing to remember)

Each ADM register block is duplicated across two "execution-environment" (EE) windows at stride `0x800`:

```
register(reg, ee) = ADM_BASE + reg_offset + ee * 0x800
```

Mainline's binding documents `qcom,ee` as the HLOS view and the driver writes there. Tenderloin's DT sets `qcom,ee = <1>` and that's matched in `drivers/dma/qcom/qcom_adm.c` (`adev->ee = 1`).

**But on this SoC, the live ADM register state actually lives at EE=0, not EE=1.** Verified by reading both windows on a running webOS system:

| Register | EE=0 readback | EE=1 readback |
|---|---|---|
| `CH_CONF` (all 16 channels) | populated with real values | **all zero** |
| `CRCI_CTL` (all 16 CRCIs) | populated for used channels | **all zero** |
| `CH_RSLT_CONF` | populated for a few channels | `0x3` (driver writes these — see below) |

Behaviour split per register:
- **`CH_CONF`** — writes to EE=1 are silently dropped. Bootloader-set values at EE=0 are what the scheduler actually obeys. Mainline driver works only because it skips the EE=1 CH_CONF rewrite entirely; the bootloader values are correct as-is.
- **`CRCI_CTL`** — same: writes to EE=1 are silently dropped. The bootloader pre-programs CRCI_CTL at EE=0 for peripherals it enables. Mainline's per-descriptor write in `adm_start_dma()` is a no-op overlay.
- **`CH_RSLT_CONF`** — writes to EE=1 *do* take effect. Driver writes `IRQ_EN | FLUSH_EN = 0x3` here at probe.
- **`CMD_PTR`** — writes to EE=1 take effect. This is the per-descriptor command-list submission, the main "do work" register.

**Practical implication for new peripheral bring-up:**
- If your peripheral uses a CRCI the **bootloader pre-programmed** (i.e. one of the SDCs, NAND, TSIF, GSBI10/HSUART2 muxed) — the driver "just works" because the EE=0 value is already correct and the runtime EE=1 write is a harmless no-op.
- If your peripheral uses a CRCI the **bootloader left blank** (read `CRCI_CTL[crci][EE=0] = 0`) — the driver will push the first burst into the peripheral's FIFO and then stall forever, because the CRCI line is never asserted to ADM. You must populate CRCI_CTL at EE=0 yourself. Use `qcom_adm_program_crci_ee0()` (defined in `drivers/dma/qcom/qcom_adm.c`, declared in `<linux/dma/qcom_adm.h>`).
- Call the helper **once at probe** while the channel is idle. Writing to a live CRCI_CTL register mid-transfer corrupts the in-flight burst (an earlier patch that did this in `adm_start_dma()` killed eMMC instantly).

---

## 3. Live Register Dump from webOS

Captured 2026-05-14 from `/tmp/admdump` (static-linked `mmap(/dev/mem)` dumper, source in `tools/admdump.c` if it survives — otherwise rebuild from `git log -p reports/`). Only non-zero entries shown.

### ADM0 (`0x18320000`)

**CH_CONF — EE=0:**
| Chan | Value | Notes |
|---:|---|---|
| 0 | `0x180008d5` | priority 8, SD=1 — same as ch1-3 |
| 1 | `0x180008d5` | |
| 2 | `0x180008d5` | **QCE CE_IN** target channel |
| 3 | `0x180008d5` | **QCE CE_OUT** target channel |
| 4 | `0x000008d6` | TSIF |
| 5–9 | `0x000008d6` | (most generic) |
| 10 | `0x000008f6` | |
| 11 | `0x000308c6` | bits 16,17 set — some "mid burst" or shadow flag |
| 12 | `0x000308c1` | |
| 13 | `0x000c08f2` | |
| 14 | `0x000c08f3` | |
| 15 | `0x000008d4` | GP / NAND_CMD path |

**CH_CONF EE=1:** all zero.

**CRCI_CTL — EE=0:**
| CRCI | Value | Peripheral |
|---:|---|---|
| 0 | `0x00010000` | (unused, but bit 16 default-set by bootloader) |
| 3 | `0x06` | NAND_DATA — 256-byte burst (blk=6) |
| 6 | `0x04` | TSIF — 192-byte burst (blk=4, matches 188-byte TS packet + 4-byte stamp) |
| 9 | `0x00040000` | GSBI10 OUT (HSUART2 mux'd) — bit 18 = `MUX_SEL` |
| 10 | `0x00040000` | GSBI10 IN — bit 18 = `MUX_SEL` |
| **4** | **`0x00000000`** | **QCE CE_IN — NOT programmed by bootloader (webOS doesn't use crypto). Driver must populate.** |
| **5** | **`0x00000000`** | **QCE CE_OUT — same.** |
| 15 | `0x06` | NAND_CMD / GP — 256-byte burst |

**CRCI_CTL EE=1:** all zero.

### ADM1 (`0x18420000`)

**CH_CONF — EE=0:**
| Chan | Value | Notes |
|---:|---|---|
| 0 | `0x000008d5` | |
| 1 | `0x0c0008d5` | NAND data — bits 18,19 set |
| 2 | `0x000008d5` | **SDC1 / eMMC RX** ← mainline mmci wires `dmas = <&adm_dma1 2>` here |
| 3 | `0x000008d5` | SDC2 |
| 4 | `0x000008d6` | SDC3 |
| 5 | `0x000008d6` | **SDC4 / eMMC TX** ← mainline mmci wires `dmas = <&adm_dma1 5>` here |
| 6 | `0x000008d6` | HSUART1 TX |
| 7 | `0x000008d6` | HSUART1 RX |
| 8–9 | `0x000008d6` | (unused) |
| 10 | `0x080208f6` | |
| 11 | `0x04022846` | bits 17, 25, 26 set |
| 12 | `0x000008c6` | |
| 13 | `0x00002842` | |
| 14 | `0x00002843` | SDC5 |
| 15 | `0x000008e4` | NAND_CMD |

**CH_CONF EE=1:** all zero.

**CRCI_CTL — EE=0:**
| CRCI | Value | Peripheral |
|---:|---|---|
| 0 | `0x00010000` | (unused, bootloader default) |
| 1 | `0x00000001` | **SDC1 (eMMC)** — 32-byte burst (blk=1) |
| 2 | `0x00000001` | SDC3 — 32-byte burst |
| 3 | `0x06` | NAND data — 256-byte burst |
| 4 | `0x00000001` | SDC2 — 32-byte burst |
| 5 | `0x00000001` | SDC4 — 32-byte burst |
| 8 | `0` | HSUART1 TX — not pre-programmed (webOS may bring up lazily) |
| 9 | `0` | HSUART1 RX — same |
| 14 | `0x00000001` | SDC5 — 32-byte burst |
| 15 | `0x06` | NAND_CMD — 256-byte burst |

**CRCI_CTL EE=1:** all zero.

---

## 4. Peripheral → DMA channel cheat-sheet

Legacy `DMOV_*_CHAN` / `DMOV_*_CRCI` macros (from `webos-linux-kernel-touchpad/arch/arm/mach-msm/include/mach/dma.h`) decoded into ADM + channel:

| Peripheral | Legacy chan | ADM | Chan | Legacy CRCI | Burst | Mainline DT phandle |
|---|---:|:---:|---:|---:|---|---|
| QCE CE_IN | 2 | ADM0 | 2 | 4 | 32 B | `<&adm_dma0 2>` (`qcom,rx-crci = <4>`) |
| QCE CE_OUT | 3 | ADM0 | 3 | 5 | 32 B | `<&adm_dma0 3>` (`qcom,tx-crci = <5>`) |
| TSIF | 4 | ADM0 | 4 | 6 | 192 B | (not in tree) |
| GSBI10 / HSUART2 TX | 8 | ADM0 | 8 | (1<<4)+9 | — | `<&adm_dma0 8>` |
| GSBI10 / HSUART2 RX | 8 | ADM0 | 8 | (1<<4)+10 | — | `<&adm_dma0 8>` |
| GP | 15 | ADM0 | 15 | — | — | — |
| NAND data | 17 | ADM1 | 1 | 3 | 256 B | (not in tree) |
| NAND data (modem) | 26 | ADM1 | 10 | 3 | 256 B | — |
| NAND data (Q6) | 27 | ADM1 | 11 | 3 | 256 B | — |
| SDC1 / eMMC | 18 | ADM1 | 2 | 1 | 32 B | `<&adm_dma1 2>` |
| SDC2 | 19 | ADM1 | 3 | 4 | 32 B | — |
| SDC3 | 20 | ADM1 | 4 | 2 | 32 B | — |
| SDC4 (eMMC TX) | 21 | ADM1 | 5 | 5 | 32 B | `<&adm_dma1 5>` |
| SDC5 | 21 | ADM1 | 5 / 14 | 14 | 32 B | — |
| HSUART1 TX | 22 | ADM1 | 6 | 8 | — | — |
| HSUART1 RX | 23 | ADM1 | 7 | 9 | — | — |
| NAND_CMD | (varies) | both | 15 | 15 | 256 B | — |

> **mux semantics:** when the legacy CRCI macro is written `(1 << 4) + N`, the high nibble flags "this CRCI is on the secondary mux path". In CRCI_CTL register terms this corresponds to bit 18 (`MUX_SEL = BIT(18)`), and we see this exact pattern on ADM0's CRCI 9/10 (HSUART2 mux = GSBI10) in the live dump.

---

## 5. CRCI_CTL register format

Single 32-bit register per CRCI, per EE window:

| Bits | Field | Meaning |
|---|---|---|
| 18 | `MUX_SEL` | 1 = use secondary mux path (e.g. GSBI10 instead of HSUART2 default) |
| 17 | `RST` | write-1-to-reset, self-clearing |
| 16 | (default-set by bootloader on CRCI 0 of both ADMs) | unknown semantic — possibly "valid" or "default route" |
| 5:0 | `blk_size` | encoded burst per CRCI assertion |

`blk_size` decoding (from `adm_get_blksize()` in `qcom_adm.c`):

| Burst bytes | `blk_size` | Used by |
|---:|---:|---|
| 8 | 1 | (legacy CE single-word) |
| 16 | 0 | — |
| 32 | 1 | **all SDCs, QCE (mainline)** |
| 64 | 2 | (some MMCI configs) |
| 128 | 3 | — |
| 192 | 4 | **TSIF** (188-byte MPEG-TS packet + 4-byte timestamp) |
| 256 | 5 — *but live shows 6 for NAND* | **NAND** |

> Note the discrepancy: the static decoder maps 256 B → `blk=5`, but the live NAND CRCI value is `0x06`. Either the encoder in `adm_get_blksize()` is off-by-one for 256-byte bursts, or NAND uses a non-standard burst (320 B?). Worth verifying if/when someone brings up mainline NAND on this SoC.

---

## 6. How to re-dump live state

The dumper source is the canonical recipe. Build with the webOS-host cross-compiler and push via novacom:

```bash
/opt/qt5-webos-sdk/gcc-linaro-4.8-2015.06-x86_64_arm-linux-gnueabi/bin/arm-linux-gnueabi-gcc \
    -std=gnu99 -static /tmp/admdump.c -o /tmp/admdump
novacom put file:///tmp/admdump < /tmp/admdump
echo 'chmod 755 /tmp/admdump && /tmp/admdump' | novacom run file://bin/sh
```

For LuneOS (mainline) instead of webOS, use a Python one-liner that mmaps `/dev/mem` — see history of `project_adm_uses_ee0_not_ee1.md` in agent memory, or the live-test snippets in earlier debug sessions.

---

## 7. References

- Legacy kernel headers: `arch/arm/mach-msm/include/mach/dma.h` in `webos-linux-kernel-touchpad/` — the `DMOV_*_CHAN/CRCI` macros are the static channel map encoded by Qualcomm.
- Legacy board file: `arch/arm/mach-msm/board-msm8x60.c` — wires QCE platform device to the CHAN/CRCI resources.
- Mainline driver: `drivers/dma/qcom/qcom_adm.c` — current EE-aware implementation.
- DT binding: `Documentation/devicetree/bindings/dma/qcom_adm.txt` — describes `qcom,ee`, `qcom,rx-crci`, `qcom,tx-crci`.
- Prior investigation memory: `project_adm_uses_ee0_not_ee1.md` (agent memory) — captures the EE=0 vs EE=1 confusion and the bisect path.
