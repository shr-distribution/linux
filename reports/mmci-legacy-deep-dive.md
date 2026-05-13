# mmci-pl18x vs legacy msm_sdcc — deep dive

**Date:** 2026-05-13
**Context:** Intermittent boot-time `DATACRCFAIL` on `mmci-pl18x` qcom variant.
Burst-width fix (`b3c0c60b56d7`) didn't eliminate it. This compares legacy
`drivers/mmc/host/msm_sdcc.c` (kernel 2.6.35-palm-tenderloin) line-by-line
against mainline `drivers/mmc/host/mmci.c` qcom variant on 6.18 to find
what's still missing.

## What already matches (verified — not the bug)

| Item | Legacy | Mainline (qcom variant) | Status |
|------|--------|-------------------------|--------|
| FIFO size | `MCI_FIFOSIZE = 64` | `fifosize = 16*4 = 64` | ✓ |
| ADM box `row_len` | `(64<<16)\|64` | `(burst<<16)\|burst` with burst=64 | ✓ (after `b3c0c60b56d7`) |
| ADM box `row_offset` | `64` (read) / `64<<16` (write) | `burst` / `burst<<16` | ✓ |
| ADM box `num_rows` | `(rows<<16)\|rows` | `(rows<<16)\|rows` | ✓ |
| CRCI for SDC1 | `CMD_SRC_CRCI(1)` / `CMD_DST_CRCI(1)` | `ADM_CMD_SRC_CRCI(1)` via DT `qcom,sdcc-crci = <1>` | ✓ |
| DMA channel | `DMOV_SDC1_CHAN = 18` | `adm_dma1 2` (= global ch 18) | ✓ |
| ADM EE selector | EE=0 (master) | `qcom,ee = <1>`, no CH_CONF rewrite (validated 2026-05-13) | ✓ |
| `MCI_CLK_FLOWENA` | set | `MCI_QCOM_CLK_FLOWENA` in `clkreg_enable` | ✓ |
| `MCI_CLK_SELECTIN` | set (feedback clock) | `MCI_QCOM_CLK_SELECT_IN_FBCLK` in `clkreg_enable` | ✓ |
| `udelay(1)` between ARGUMENT and COMMAND writes | yes (via `msmsdcc_delay`) | yes (gated by `qcom_datactrl_delay`) | ✓ |
| `udelay(5)` after DATATIMER/DATALENGTH, before DATACTRL | yes | yes (gated by `qcom_datactrl_delay`) | ✓ |
| `udelay(5)` after DATACTRL | yes | yes (gated by `qcom_datactrl_delay`) | ✓ |

## What differs (candidates for the DATACRCFAIL bug)

### 1. ⚠️ MISSING — `udelay(50)` between MMCICLOCK and MMCIPOWER writes

**Legacy `msm_sdcc.c:1171-1177`:**
```c
writel(clk, host->base + MMCICLOCK);
udelay(50);
if (host->pwr != pwr) {
    host->pwr = pwr;
    writel(pwr, host->base + MMCIPOWER);
}
```

**Mainline `mmci.c:2362-2374`:**
```c
mmci_set_clkreg(host, ios->clock);     /* writes MMCICLOCK */
mmci_set_max_busy_timeout(mmc);
mmci_write_pwrreg(host, pwr);          /* writes MMCIPOWER — NO delay */
mmci_reg_delay(host);                  /* delay AFTER both writes */
```

**Why this matters:** During card init the controller transitions
`MCI_PWR_OFF → MCI_PWR_UP → MCI_PWR_ON`. Each step requires MMCICLOCK to
have stabilised first. Mainline writes both registers back-to-back with
zero MCLK cycles in between. Legacy waits 50 µs explicitly. This is a
classic "intermittent CRC at boot" smell — works when the system is fast
enough that the bus arbitration accidentally gives enough gap, fails when
something else loads the bus.

### 2. ⚠️ MISSING — clock-stabilisation delay after `clk_set_rate`, before MMCICLOCK write

**Legacy `msm_sdcc.c:1115-1124`:**
```c
if (ios->clock != host->clk_rate) {
    rc = clk_set_rate(host->clk, ios->clock);
    host->clk_rate = ios->clock;
}
/*
 * give atleast 2 MCLK cycles delay for clocks
 * and SDCC core to stabilize
 */
msmsdcc_delay(host);     /* 1 + 3 * USEC_PER_SEC / clk_rate */
clk |= MCI_CLK_ENABLE;
```

`msmsdcc_delay` = `udelay(1 + 3*1e6/clk_rate)`. At 48 MHz that's just
1 µs, at 400 kHz it's 8 µs.

**Mainline `mmci.c:2349-2365`:** runs `clk_set_rate` then immediately
calls `mmci_set_clkreg`. No delay.

**Why this matters:** the SDC core needs ≥ 2 MCLK cycles after the source
RCG rate changes before MMCICLOCK can be safely reprogrammed. Without
this, the rate change and the register write race; the controller may
sample a half-changed clock during the MMCICLOCK update and end up in an
indeterminate state. Manifests as DATACRCFAIL on the first transfer
after a rate change.

### 3. ⚠️ MISSING — mid-frequency snap-down to 24 MHz

**Legacy `msm_sdcc.c:1111-1113`:**
```c
if ((ios->clock < host->plat->msmsdcc_fmax) &&
    (ios->clock > host->plat->msmsdcc_fmid))
    ios->clock = host->plat->msmsdcc_fmid;
```

With `msmsdcc_fmid = 24 MHz`, any request in the `(24, 48)` MHz range
gets snapped down to 24 MHz. Only exact-48 MHz requests reach the
controller at full speed.

**Mainline:** no equivalent. Whatever the MMC core requests is passed
straight to `clk_set_rate`.

**Why this matters:** `clk_tbl_sdc[]` (`drivers/clk/qcom/gcc-msm8660.c:1599`)
has discrete rates `{ 144 kHz, 400 kHz, 16/17.07/20.21/24 MHz, 48 MHz }`.
Nothing between 24 MHz and 48 MHz. If the MMC core requests, e.g.,
26 MHz (some MMC HS modes), the qcom clock RCG `round_rate` callback
picks the closest match — which is implementation-dependent but is
plausibly 48 MHz (round-up) or 24 MHz (round-down). If it rounds up to
48 MHz BEFORE the controller is configured for HS-MMC, signals will be
miss-sampled and we get DATACRCFAIL.

The legacy approach is conservative: never run between 24 and 48; either
take the safe-step 24 MHz or full 48. This avoids the round-up ambiguity.

### 4. ℹ️ MISSING — `MCI_CLK_PWRSAVE` bit when clock > 400 kHz

**Legacy `msm_sdcc.c:1135-1136`:**
```c
if (msmsdcc_is_pwrsave(host))
    clk |= MCI_CLK_PWRSAVE;
```

`msmsdcc_is_pwrsave` returns 1 iff `clk_rate > 400 kHz`.

**Mainline `mmci.c:509`:** `/* clk |= MCI_CLK_PWRSAVE; */` — explicitly
commented out.

**Why this matters:** PWRSAVE is the auto-clock-gating bit. When set,
the controller stops the SDC clock between transfers (a power
optimisation). When clear (mainline default), the clock runs free.
Unlikely to cause DATACRCFAIL on its own — but matters for power draw.
A free-running clock could also potentially expose interference with
other system activity that's gated by clock edges; this is speculation.

### 5. ℹ️ DIFFERENT — DFAB bandwidth voting

**Legacy:** explicit `clk_set_rate(host->dfab_pclk, 64000000)` —
forces a 64 MHz vote on the Daytona Fabric clock for as long as the
controller is up.

**Mainline:** uses `icc_set_bw(host->icc_path, 0, 400000)` for 400 MB/s
peak via the interconnect framework, with bandwidth path expressed as
`<&daytona_fabric DFAB_MAS_SDC1 &apps_fabric AFAB_SLV_EBI_CH0>`.

**Why this matters:** the interconnect framework eventually translates
the 400 MB/s vote into an RPM resource request that sets the DFAB
clock rate. If that translation table doesn't include a path to ≥ 64 MHz
DFAB, the controller could be operating with insufficient peripheral
fabric throughput → CMDTIMEOUT during card init when the bus is
contented. We've seen `CMDTIMEOUT cmd52 / cmd5 / cmd55` chains at boot,
which is exactly the pattern this could explain.

Mainline's interconnect-derived DFAB rate at boot would be worth dumping
from `/sys/kernel/debug/clk/dfab_clk/clk_rate` to compare against legacy.

## Recommended fix series (in priority order)

Each as an independent commit per the project's commit-discipline rule.

### Patch 1 — `udelay(50)` between MMCICLOCK and MMCIPOWER writes (most likely fix)

In `mmci.c::mmci_set_ios`, between `mmci_set_clkreg(host, ios->clock)`
and `mmci_write_pwrreg(host, pwr)`, add (gated on `qcom_datactrl_delay`
since that's already the per-variant "qcom needs extra timing" flag):

```c
if (host->variant->qcom_datactrl_delay) {
    /*
     * Legacy msm_sdcc waits 50us between MMCICLOCK and MMCIPOWER
     * writes during card init. Without this delay, intermittent
     * DATACRCFAIL on cold boots (~1/4 boots).
     */
    udelay(50);
}
```

### Patch 2 — Clock-stabilisation delay after `clk_set_rate`

In `mmci.c::mmci_set_ios`, immediately after the `clk_set_rate(host->clk, ...)`
block (around line 2357):

```c
if (host->variant->qcom_datactrl_delay && ios->clock) {
    /*
     * Legacy msm_sdcc waits "atleast 2 MCLK cycles" after
     * clk_set_rate before reprogramming MMCICLOCK. Use the same
     * formula: 1 + 3000000/clk_rate microseconds (= 8us at 400kHz,
     * 1us at 48MHz).
     */
    udelay(1 + 3000000 / ios->clock);
}
```

### Patch 3 — Mid-frequency snap-down (defensive, may not be needed)

If patches 1+2 don't fully resolve, add a per-variant clock snap-down
hook. Implementation: add `clk_snap_below` field to `variant_data` set
to `24000000` for qcom; in `mmci_set_clkreg` / `set_ios`, clamp:

```c
if (variant->clk_snap_below && ios->clock < host->mmc->f_max &&
                                ios->clock > variant->clk_snap_below)
    ios->clock = variant->clk_snap_below;
```

### Patch 4 — `MCI_CLK_PWRSAVE` for qcom (deferred — power not correctness)

Add `MCI_CLK_PWRSAVE` to `variant_qcom.clkreg_enable` once cards run
> 400 kHz. Track that transition state. Power optimisation only —
defer until after correctness is solid.

## Open questions for next live capture

When the device boots cleanly again (post-fsck), capture:

```sh
cat /sys/kernel/debug/clk/clk_summary | grep -E "sdc1|dfab|sfab|afab"
cat /sys/kernel/debug/clk/sdc1_clk/clk_rate
cat /sys/kernel/debug/clk/dfab_clk/clk_rate
cat /sys/kernel/debug/mmc0/ios
```

Compare against the legacy webOS readback (need to capture from a clean
webOS boot — `dmesg | grep "MMC clock"` already shows `400000 ->
48000000 Hz`, but the live debugfs dump would confirm DFAB pclk too).

If DFAB on mainline shows < 64 MHz under load → Patch 5: bump
interconnect bandwidth vote or add a direct clock vote.
