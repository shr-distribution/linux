# ADM DMA on HP TouchPad (APQ8060/MSM8660): eMMC + WiFi (+BT) Contention — Status & Everything Tried

**Audience:** external reviewer (Gemini) for fresh input.
**Date:** 2026-05-25 (Rev 3 — adds §6c: legacy-derived ICC bandwidth values + the landed `avg_bw` fix and starvation diagnostics. Rev 2 added §6b code-grounded re-analysis re-prioritizing toward fabric bandwidth).
Kernel: mainline Linux 6.18 port, branch `tenderloin/6.18/upstream-patches`.
**One-line problem:** Two (now three) DMA masters share a single ADM controller + IRQ. Each works in isolation, but **concurrent eMMC DMA + WiFi DMA (and now BT-over-DMA UART) intermittently corrupt each other** → eMMC `DATACRCFAIL` / "error during DMA transfer", WiFi probe `-110`, sometimes rootfs remounts read-only. We have NOT found a robust fix for the concurrency; only PIO+PIO was ever 100% stable.

---

## 1. Hardware topology

SoC: Qualcomm APQ8060 (MSM8660 family), dual Scorpion ARMv7 @ 1.5 GHz.

**ADM (Application Data Mover)** — Qualcomm's pre-BAM datamover. Two instances:
- ADM0 @ `0x18320000` — crypto (CE), one debug UART.
- **ADM1 @ `0x18420000`** — the contended one. Single hardware IRQ for the SD=1 (AARM) security domain (IRQ 167).

**Consumers on ADM1 (all share the one ADM1 IRQ):**

| Consumer | Linux dev | Controller | ADM1 channel | CRCI | Bootloader priority (EE=0) |
|---|---|---|---|---|---|
| eMMC | mmc0 | sdcc1 @ `0x12400000` | chan 2 (rx+tx) | **CRCI 1** | 5 |
| WiFi (AR6003 SDIO) | mmc1 | sdcc4 @ `0x121c0000` | chan 5 (rx+tx) | **CRCI 5** | 6 |
| Bluetooth (CSR BlueCore, BCSP) | gsbi6 serial @ `0x16540000` | msm_serial | chan 7 | CRCI 8/9 | — |

DTS (current):
```
&sdcc1 { dmas = <&adm_dma1 2>, <&adm_dma1 2>; qcom,sdcc-crci = <1>; };   /* eMMC */
&sdcc4 { dmas = <&adm_dma1 5>, <&adm_dma1 5>; qcom,sdcc-crci = <5>;
         qcom,dummy52-required; };                                       /* WiFi */
gsbi6_serial { dmas = <&adm_dma1 7> ... };                               /* BT UART (currently being toggled DMA<->PIO) */
```

ADM1 → SFAB → AFAB → EBI (system memory). The fabric (DFAB/AFAB) bandwidth is RPM-owned; HLOS cannot read the live rate via /dev/mem.

**CH_CONF / CRCI_CTL live register window is EE=0** (verified by /dev/mem dump from webOS 2.6.35). EE=1/2/3 read zeros. Bootloader programs ch2=pri5, ch5=pri6, both SD=1. We do NOT reprogram CH_CONF (bootloader covers it); legacy webOS wrote it at the dead EE=1 window (i.e. legacy's CH_CONF writes were no-ops — priority came from bootloader).

## 2. Software: mainline vs legacy

- **Mainline:** `drivers/dma/qcom/qcom_adm.c` (dmaengine/virt-dma framework) + `drivers/mmc/host/mmci.c` (PL180/mmci-pl18x with a qcom variant).
- **Legacy webOS 2.6.35-palm:** `arch/arm/mach-msm/dma.c` (`msm_dmov`) + `drivers/mmc/host/msm_sdcc.c`. The reference we keep comparing against.

Key architectural difference we keep running into: **webOS `msm_datamover_irq_handler` is a featherweight hardirq** — it runs `complete_func` (which for SDCC just `tasklet_schedule()`s) + writes the next CMD_PTR, nothing else. Mainline `adm_dma_irq` goes through `vchan_cookie_complete` + `adm_start_dma` (re-arm) which is **~3× heavier** (measured ~4–6 µs vs ~1.5–2 µs per completion on Scorpion @ 1.5 GHz).

## 3. What WORKS today (in isolation, and one validated concurrent point)

Through a long campaign we got each path working **on its own**:

- **WiFi end-to-end (when it probes):** AR6003 BMI firmware upload, HTC, WMI connect, `wlan0` registers, `iw scan` returns ~18 APs, WPA2 associate to an AP (-63 dBm), static IP. Firmware: `ar6003 hw 2.1.1 sdio fw 3.2.0.144`. So the WiFi stack and DMA bulk path are fundamentally correct.
- **eMMC in isolation:** boots, mounts, `dd` reads at ~22–29 MB/s, 8-bit bus @ 48 MHz HS.
- **One historically-validated concurrent config (2026-05-20):** "CPU1 IRQ pin + hardirq-only completion + hardirq shaving" gave **29.2 MB/s eMMC sustained (256 MB read) with concurrent WiFi scan, zero DATACRCFAIL**. We have NOT been able to reproduce that reliably since (see §6 — possibly eMMC FS now degraded, or methodology differed).

## 4. The fix campaign (chronological, with commit hashes)

These are the real, kept fixes that got each path working. All WiFi-specific ones are **gated to mmc1** (eMMC is mmc0 — applying WiFi workarounds to it destabilised eMMC, hard lesson).

1. **DMA-read completion callback** (`67fb82423d96`): On MSM8660 the SDCC DPSM does **not** raise `MCI_DATAEND` for a DMA *read* — only `MCI_DATABLOCKEND` (which is masked). `mmci_data_irq` completes only on DATAEND, so DMA reads hung forever. Fix: wire `desc->callback = mmci_qcom_dma_complete` for mmc1 reads, completing from the ADM IRQ like legacy's `complete_func`. (Writes DO get DATAEND.)

2. **Honor latched errors in the callback** (`b6a80fadbb26`): sample MMCISTATUS in the callback and honor DATACRCFAIL/RXOVERRUN/DATATIMEOUT before declaring success (else risk returning CRC-bad data on the DMA-done edge).

3. **`validate_dma` = legacy rule** (`f52bdcc13229`): legacy only DMAs transfers `>= FIFO(64B) && multiple-of-64`; everything else PIO. Mainline's `dma_threshold=32` wrongly routed small BMI control transfers through DMA, leaving the SDCC data path dirty. Now: DMA iff `len>=64 && len%64==0` (mmc1).

4. **CRCI block-size = 1** (`bcecbd6238ab`, later re-established as `3c2dce9b7445` after being lost in a `git reset`): legacy `adm1_crci_conf[]` hardcodes `blk_size=1` (half-FIFO, 32B = SDCC half-full CRCI trigger) for **both** SDCC CRCIs (1 and 5). Mainline `adm_get_blksize(burst)` computes 2 (full-FIFO) for 64B — this **mis-paces the CRCI handshake vs the SDCC FIFO**, and the drift accumulates over long transfers → `DATACRCFAIL`/`RXOVERRUN` on large reads (eMMC 128 KB reads CRC-fail ~46 KB in; WiFi 128B FIXED reads error). Fix: force `blk_size=1` for CRCI 1 & 5 only (crypto CE untouched).

5. **PROG_DONE wait after WiFi DMA writes** (`67e9181954b3`): a DMA write's DATAEND fires as soon as data is in the FIFO, but the card still owes its programming-busy/CRC token (`MCI_QCOM_PROGDONE`, bit 23). The next CMD53 then raced ahead of AR6003 programming → CMDTIMEOUT. Fix (mmc1 writes): poll MMCISTATUS for PROG_DONE before completing.

6. **Defer WiFi DMA-read completion to the ADM callback, ignore DATAEND** (`ca28da04406e`): the "reads never get DATAEND" premise (#1) is only *sometimes* true. For small/fast reads, **DATAEND fires while the ADM is still draining the CRCI handshake** (the ADM completion callback arrives ~8 ms *later* — "callback LATE"). Completing on DATAEND tears down the DPSM mid-ADM-drain → SDCC **CPSM stuck in data-state response window** → the *next* command never completes (even a dummy CMD52 gets no IRQ). Fix: for mmc1 DMA reads, do NOT complete on DATAEND — let the ADM callback (which fires when the descriptor is truly done) complete it. This is exactly legacy (reads complete only from `complete_func`, never DATAEND).

7. **Clear `atomic_submit` before the dummy52 CMD52** (`ea3a8d1d8b78`): `mmci_should_atomic_submit` sets `host->atomic_submit.active=true` for WiFi DMA reads, cleared only at the top of `__mmci_start_request`. The dummy52-drain dispatch path bypasses that and calls `mmci_start_command` directly → with `active` still set, the CMD52 ARG/CMD got **stashed for the ADM exec_func instead of written to MMCICOMMAND** → never issued → no IRQ → D-state hang. Fix: clear atomic_submit in the dummy52 path.

8. **eMMC initcall revert to mainline** (`9e3cc8d29b8b`): the port had moved mmci to `subsys_initcall` and `mmc_blk` to `subsys_initcall_sync` (faster rootfs). But `mmci_probe` votes DFAB fabric bandwidth via `of_icc_get("sdc") + icc_set_bw(512 MB/s)`, and the MSM8660 NOC interconnect **provider** registers at `subsys_initcall_sync` — *after* mmci's `subsys_initcall`. So `of_icc_get` returned `-EPROBE_DEFER` and the bandwidth vote only landed via a deferred-probe retry whose timing raced the provider → some boots eMMC ran under-bandwidthed → DATACRCFAIL. Reverted mmci to mainline `module_amba_driver` (device_initcall, after the provider). This fixed boot-time reliability (no more aborted-journal-at-boot) but NOT the under-load contention.

**Result of 1–8:** WiFi probes + scans + associates; eMMC boots clean and reads fast — **each in isolation**.

## 5. The concurrency-mitigation work (the actual hard problem)

This is where it's unresolved. The single ADM1 IRQ + heavy mainline completion path means one channel's completion servicing blocks the other's.

**Validated truth table (all 4 combos tested on-device 2026-05-20):**

| Config | eMMC under concurrent WiFi | WiFi BMI |
|---|---|---|
| CPU1-pin + tasklet-deferred callback | 5.7 MB/s, **DATACRCFAIL** + bus-width fallback | OK |
| no-pin + tasklet | 1.1 MB/s (everything on CPU0) | OK |
| no-pin + hardirq-only | 29.4 MB/s | **FAILS** ("failed to read reg table") |
| **CPU1-pin + hardirq-only + shaving** | **29.2 MB/s sustained 256 MB** | **OK** |

Kept mitigations:
- **Pin ADM1 IRQ to CPU1 + `IRQF_NOBALANCING`** (`a57123d66e21`): isolates the heavy ADM hardirq on CPU1 so sdcc4's PIO-refill IRQ on CPU0 doesn't miss its DPSM window during AR6003 BMI.
- **Hardirq path shaving B+F+G** (`8622c90390fb`): `__ffs` to iterate only set channel bits; plain `spin_lock` (already in hardirq) instead of `spin_lock_irqsave`; `dma_wmb()` instead of full `wmb()`. ~800 ns–1.2 µs saved.
- **Hardirq-only completion**: complete cookie + re-arm next DMA + run consumer callback all synchronously in the ADM hardirq (NO tasklet deferral).
- **AR6003 post-BMI settle + CMD52 wakeup poll** (`b40ed0d52a0c`): the chip enters deep sleep mid-firmware-entry if SDIO clock jitters (fabric contention) coincide with the boot-ROM→SRAM jump; CMD52 polling keeps its SDIO state machine awake.

## 6. CURRENT STATUS (2026-05-25) — regressed / not reproducible

The 29.2 MB/s concurrent result is **not currently reproducible**. Symptoms now:
- WiFi probe **intermittently fails** with `mmci 121c0000.mmc: error during DMA transfer!` → `ath6kl: Failed to start hardware: -110` on the BMI/htc_start CMD53. Succeeds when eMMC is quiet, fails when eMMC is busy during the BMI burst.
- eMMC throws `DATACRCFAIL` on large reads under concurrent WiFi; HW card-reset fails (`-110`); ext4 journal aborts; rootfs remounts **read-only** → userspace binaries fail to `exec` (their pages demand-fault from the failing eMMC, even when copied to tmpfs — the .so closure pages fault back → `Bus error`).

**Things falsified this week (important — don't re-suggest):**
- **Deferring the ADM callback to a tasklet to mimic webOS** (commit `75f1dbf904e6` "Fix #7"): re-introduced the falsified "pin + tasklet" combo. We reverted it, but then **hardirq-only ALSO failed the WiFi probe with -110** — so tasklet-vs-hardirq is NOT the deciding factor for the *current* breakage. (A single probe pass/fail was wrongly used as an A/B signal; the failure is racy — need N iterations.)
- BT-over-DMA UART (gsbi6 on ADM1 chan 7) running BCSP made it dramatically worse — continuous BCSP SYNC-retry DMA on ADM1 broke both eMMC and WiFi. Moving **BT to PIO** removed *that* contention (eMMC boot errors 270→6) but eMMC↔WiFi concurrency still fails intermittently.
- dummy52 cannot drain a DMA-read-wedged CPSM (the CMD52 itself never completes when the CPSM is stuck).
- CH_CONF priority writes — bootloader already sets WiFi pri6 > eMMC pri5; not the lever.

## 6b. Round-2 analysis (external review + code verification) — reframes diagnosis toward FABRIC bandwidth

An external reviewer (Gemini) argued the `DATACRCFAIL` is an **IRQ-latency / descriptor re-arm gap** problem: at 48 MB/s an empty 64 B FIFO overruns in ~1.33 µs (only ~666 ns of slack from the half-full CRCI trigger), so a 4–6 µs mainline ADM hardirq between descriptor boundaries guarantees an overrun. The reviewer recommended **Candidate D (CMD_PTR staging)** as the fix.

**We then verified the actual `qcom_adm.c` chaining model, and it inverts that conclusion:**

- **One `adm_prep_slave_sg` builds ONE command-pointer list (CPL)** covering *all* scatterlist segments as chained box descriptors (`for_each_sg` loop, terminated by `ADM_CPLE_LP`).
- **`adm_start_dma` writes ONE `CMD_PTR`** for that whole CPL.
- The IRQ's `adm_start_dma` re-arm targets the **next queued mmc *request*** (`curr_txd` / `vchan_next_desc`), **not** per-segment within a transfer.

⇒ For a single multi-block read (one CMD18 → one SG list → one CPL → one CMD_PTR), **the ADM runs the entire transfer autonomously in hardware, paced by CRCI, with NO CPU intervention until completion.** The CPU is not on the intra-transfer critical path.

**Therefore the 666 ns FIFO-drain deadline must be met by the ADM *hardware*, not the CPU.** The 4–6 µs IRQ latency cannot cause the `DATACRCFAIL` at byte 84992/131072 — that point is mid-hardware-descriptor where the CPU isn't involved. The only way the ADM misses the drain deadline is if **it** is starved: it cannot write drained bytes out to EBI because the **SFAB→AFAB→EBI fabric is congested by concurrent WiFi DMA**, or AHB/CRCI arbitration stalls it. The partial-then-trailing-CRC16-fail signature fits this exactly (ADM back-pressures → SDCC FIFO, still being firehosed by the card, overruns → byte shift → trailing CRC fails).

**Conclusions from the code-grounded re-analysis:**
- The intra-transfer eMMC `DATACRCFAIL` under WiFi load is most likely **fabric bandwidth / AHB arbitration starvation of the ADM**, NOT CPU IRQ latency.
- **Candidate D would only shrink the gap *between* separate mmc requests** — and during that gap the card isn't streaming (CMD18 done, next CMD53 not yet issued), so there's nothing to overrun. **Candidate D is NOT expected to fix the intra-transfer CRCFAIL; deprioritized.**
- The **CPU1 IRQ pin is a separate axis**: it fixes the **WiFi BMI** failure, where WiFi runs in *PIO* and the sdcc4 PIO-refill IRQ on CPU0 was blocked by the heavy ADM hardirq. That is genuine CPU-latency contention — but on the WiFi-PIO path, not the eMMC-DMA-overrun path. (So both can be true: pin needed for WiFi-BMI-PIO, fabric bandwidth needed for eMMC-DMA.)
- This makes the **DFAB clock vote the prime suspect** for the eMMC side.

## 6c. Legacy-derived ICC bandwidth values + the landed fix (Rev 3)

We derived the correct fabric vote from legacy webOS rather than guessing:

- **SDCC (load-bearing):** `msm_sdcc.c:1738` does `clk_set_rate(host->dfab_pclk, 64000000)` + a persistent `clk_enable` on **every active SDCC** (`board-tenderloin.c`: `pclk_src_dfab=1` on SDC1 and SDC4; `fmax=48 MHz`). 64 MHz on the 64-bit DFAB = 64e6 × 8 = **512 MB/s = 512000 kBps**, held **sustained** (not a transient ceiling).
- **ADM (minor):** `dma.c:755` does `clk_set_rate(ebi1_adm_clk, 27)` — a minimal EBI **keep-alive**, not a heavy data vote. The heavy lifting was the per-SDCC DFAB=64 MHz.

Mainline confirmation: the msm8660 ICC provider (`drivers/interconnect/qcom/msm8660.c`) uses **buswidth = 8** for all fabric nodes, and `icc-rpm.c` sets `agg_rate = max(avg_rate, peak_rate)` then `clk_set_rate(bus_clk, rate)` with `rate = bw/8`. So **512000 kBps ⇒ 64 MHz DFAB exactly** — the value is *derived*, not invented.

**The bug:** both consumers voted `avg_bw = 0` (peak-only):
```
mmci.c:3438     icc_set_bw(host->icc_path, 0, 512000);   // peak only
qcom_adm.c:1431 icc_set_bw(adev->icc_path, 0, 128000);   // peak only
```
`max(avg,peak)` yields 64 MHz in the **ACTIVE** state, but `avg=0` lets the floor lapse across RPM active/sleep context transitions — unlike legacy's persistent `clk_enable` hold.

**Landed change (commit `bc2998b4883f`):**
```
mmci.c     icc_set_bw(path, 512000, 512000)   // sustained 64 MHz DFAB = legacy hold
qcom_adm.c icc_set_bw(path, 128000, 128000)   // sustained EBI keep-alive (>= legacy)
```
Plus diagnostics for the next failing run:
- `mmci` `DIAG[DATACRCFAIL]` now decodes latched error bits by name — **`RXOVERRUN` set ⇒ ADM/fabric drain starvation** (vs card-side `DATATIMEOUT`).
- `qcom_adm` logs `ADM_CH_RSLT` + `FLUSH_STATE0` at warn level for the SDCC channels (ch2=eMMC/CRCI1, ch5=WiFi/CRCI5), so an ADM FLUSH coincident with an SDCC RXOVERRUN is visible by timestamp.

**Caveat (still honest):** because `max(avg,peak)` already gives 64 MHz in ACTIVE state, this `avg_bw` change may not be *the* cure if the ACTIVE-state DFAB was already 64 MHz under load. If we still starve at a confirmed 64 MHz DFAB, then 512 MB/s of fabric is not the raw limit and the cause is arbitration / CRCI context-switching (the erratum lead) — which the RXOVERRUN-vs-clean-ADM capture will settle. **Not yet tested on-device** (pending e2fsck + clean boot).

## 7. Open questions / candidate directions (re-prioritized after §6b)

1. **PRIME SUSPECT — DFAB/fabric bandwidth.** Legacy force-votes DFAB=64 MHz (`pclk_src_dfab=1`) on both SDCC. Mainline relies on `mmci_probe`'s `of_icc_get("sdc") + icc_set_bw(512 MB/s)` resolving via the NOC provider. Is the *live* DFAB rate actually ≥64 MHz under concurrent WiFi load, or does RPM drop it / does the vote not land? Can't read it from HLOS (/dev/mem = 0, RPM-owned), so **A/B it**: raise the ICC bandwidth floor on both SDCC nodes (and/or on the ADM's own `interconnects` path ADM→SFAB→AFAB→EBI) and measure whether the CRCFAIL rate drops. This directly tests the fabric hypothesis.
2. **Disambiguation that actually separates fabric vs latency** (the eMMC-clock-slowdown test does NOT — it relaxes *both*): 
   - Vary WiFi **throughput** vs WiFi **IRQ rate** independently — heavy WiFi bulk (high fabric, moderate IRQ) vs many tiny WiFi transfers (high IRQ, low fabric). If eMMC CRCFAIL tracks throughput → fabric; if it tracks IRQ count → latency.
   - Capture ADM `RESULT` / `FLUSH_STATE0` at the failing transfer (driver already logs FLUSH) — ADM-side flush/error vs SDCC-side RXOVERRUN tells us which side starved.
3. **Throttle sdcc1 `max_seg_size` / `max_blk_count`** so no single ADM transfer is long enough to starve mid-flight under concurrent WiFi (caps the exposure window even if fabric is the issue).
4. **Serialize the two SDCC DMA submissions during contention windows** — in-kernel arbiter between sdcc1/sdcc4 ADM submissions, or quiesce eMMC writeback during WiFi BMI. (Quiesce-then-probe experiment started, not completed cleanly.)
5. **Pre-stage CMD_PTR on submit** (candidate "D"): DEPRIORITIZED per §6b — only affects inter-request gaps where the card isn't streaming. Keep as a fallback only if (2) somehow points back at inter-request latency.
6. **Does per-CRCI `blk_size` interact with concurrent CRCI 1 + CRCI 5 traffic** in a way the isolated test missed? Both at half-FIFO (32 B) trigger — do simultaneous half-full triggers on the shared ADM overrun? (Lower priority than fabric.)
7. **Clean baseline first:** the eMMC FS is degraded from dozens of RO/wedge cycles (ext4 error count 120). `e2fsck` before any measurement — filesystem-layer retry/fault delays poison concurrency numbers.

## 8. Concrete asks for the reviewer (Round 2 — focus on fabric, given §6b)

Round-1 framed this as IRQ latency; §6b's code verification (ADM hardware-chains the whole SG list, CPU off the intra-transfer path) reframes it as **fabric/AHB bandwidth starvation of the ADM**. Please pressure-test that and advise:

- Given that the ADM hardware-chains the entire multi-block transfer (one CMD_PTR, autonomous CRCI-paced draining, no CPU mid-transfer), do you agree the intra-transfer `DATACRCFAIL` (fails at DATACNT=84992/131072, clock correct) is **ADM-can't-drain-to-EBI starvation** rather than CPU IRQ latency? If not, what mechanism keeps the CPU on the critical path once the descriptor is running?
- On MSM8660, the SDCC peripheral clock can be sourced from the **Daytona Fabric (DFAB)**; legacy force-votes DFAB=64 MHz via `pclk_src_dfab=1`. If the ADM's read drain and the SDCC's own clock both depend on fabric that RPM may down-clock under load, **what's the right way to pin/floor DFAB (or the ADM→EBI ICC path) from mainline** so the drain never starves? Is an `icc_set_bw` floor sufficient, or does DFAB need an explicit clock vote the interconnect framework doesn't model?
- For the partial-then-trailing-CRC16 signature on a CRCI-flow-controlled read: does that distinguish **ADM-side drain starvation** (ADM can't write to memory) from **SDCC-side RXOVERRUN** (FIFO overflow), and which ADM `RESULT`/`FLUSH_STATE0` bits would confirm which side stalled?
- Did webOS in practice **avoid true eMMC-DMA + WiFi-DMA concurrency** (e.g. I/O-scheduler serialization, WiFi mostly idle during bulk storage), rather than truly arbitrating both on ADM1 simultaneously?
- Any MSM8660-specific ADM/fabric erratum around concurrent CRCI flow-control or SFAB→AFAB→EBI arbitration under dual-master load?
- Sanity-check the proposed experiment ladder in §7 (DFAB/ICC floor A/B → throughput-vs-IRQ-rate separation → `max_seg_size` cap) — is that the right order, and what are we missing?

## 9. Reference pointers (in-tree)
- Driver: `drivers/dma/qcom/qcom_adm.c`, `drivers/mmc/host/mmci.c`.
- DTS: `arch/arm/boot/dts/qcom/qcom-apq8060-tenderloin-common.dtsi` (sdcc1 @ line ~2427, sdcc4 @ ~2538, gsbi6 serial @ ~2988), base nodes in `qcom-msm8660.dtsi`.
- Live webOS register ground truth: `reports/adm-investigation/adm-live-ee0-ground-truth.md`.
- Legacy reference: webOS 2.6.35-palm `arch/arm/mach-msm/dma.c` + `drivers/mmc/host/msm_sdcc.c`.
