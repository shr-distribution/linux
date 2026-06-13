Legacy webOS msm_bus_fabric live arb capture (Topaz, 2026-06-13)
=================================================================

Test rig: HP TouchPad Topaz WiFi, webOS 3.0.5, msm_bus_fabric vendor
driver, instrumented kernel with debugfs mounted at /sys/kernel/debug.
WiFi associated to 5 GHz "HerrieVlada" at 121-150 Mbit/s, -51 dBm.

The capture was used to refute an earlier static-source-code reading
of the legacy msm_bus_board_8660.c gateway nodes (which lack `.tier`
fields) and to authoritatively answer the question "what does legacy
actually write into the RPM arb tables under load?".

----------------------------------------------------------------------
1. AFAB live arb (msm_apps_fab)
----------------------------------------------------------------------

Idle / display-only:

  BWSum:
    0x9f4  0x0  0x0  0x0           ← slv_ebi_ch0 = 0x9f4 (~318 MB/s)
  Arb TSlave 0 (EBI_CH0 row):
    0x0   0x0   0x89f4   0x0       ← master port 2 = TIER1+bw
  Arb TSlave 1:
    0x0   0x0   0x0      0x0

Under 100 MB WiFi DL:

  BWSum:
    0x45cb 0x0  0x0  0x0           ← slv_ebi_ch0 = 0x45cb (~2.2 GB/s)
  Arb TSlave 0 (EBI_CH0 row):
    0x0   0x0   0xc5cb   0x0       ← master port 2 = TIER1 + larger bw
  Arb TSlave 1:
    0x0   0x0   0x0      0x0

Decoding (legacy mainline-aligned format):
  Bit 15 (0x8000)    : TIER1 priority indicator
  Bits 14:0          : bandwidth in 128 KB/s units
  0x89f4 = TIER1 | 0x09f4   (2548 * 128 KB/s = ~318 MB/s)
  0xc5cb = TIER1 | 0x45cb   (17867 * 128 KB/s = ~2.2 GB/s)

Master port mapping on AFAB (mainline enum + legacy verified):
  port 0 = AFAB_MAS_AMPSS_M0 (CPU core 0 IF)
  port 1 = AFAB_MAS_AMPSS_M1 (CPU core 1 IF)
  port 2 = AFAB_TO_MMSS gateway (MMFAB-side traffic landing on AFAB)
  port 3 = AFAB_TO_SYSTEM gateway (SFAB-side traffic landing on AFAB)

Findings:
  - port 2 (afab_to_mmss) is TIER1 in EVERY snapshot, regardless of
    SDC / WiFi activity.  The TIER1 elevation is sourced from MMSS
    consumers (MDP, GPU) voting against slv_ebi_ch0; the path
    traversal propagates TIER1 + bw onto the gateway's master arb
    cell on AFAB.
  - port 3 (afab_to_system) is ZERO in every snapshot.  Legacy SDC
    and ADM drivers do not vote msm_bus_scale -- they drive EBI
    through clk_set_rate on ebi1_clk / dfab_clk directly.  So nothing
    ever accumulates into afab_to_system's arb cell.
  - ports 0 and 1 (AMPSS) are ZERO.  cpufreq adjusts ebi1_clk via the
    clock tree, not via arb voting.

Implication for mainline parity:
  - afab_to_mmss MUST keep ARB_TIER1.  Reverting to TIER2 (commit
    917ed2f3797e) was contradicted by this live data.
  - afab_to_system MUST NOT write its (mainline-only) propagated
    TIER2 bw into the EBI_CH0 arb row.  mainline mmci + qcom_adm
    use icc_set_bw and would generate phantom contention.

----------------------------------------------------------------------
2. MMFAB live arb (msm_mm_fab)
----------------------------------------------------------------------

Display-only:

  BWSum:
    0x9f4   0x9f4   0x0   0x0      ← SMI = APPSS gateway = 0x9f4
  Arb TSlave 0 (SMI row):
    0x89f4 0x0 0x0 0x0 0x0 0x0 0x0 0x0 0x0 0x0 0x0 0x0 0x0 0x0
    ^^^^^^
    port 0 = MDP_PORT0 (TIER1+bw)
  Arb TSlave 1 (APPSS gateway row):
    0x89f4 0x0 0x0 0x0 0x0 0x0 0x0 0x0 0x0 0x0 0x0 0x0 0x0 0x0
    ^^^^^^
    port 0 = MDP_PORT0 (TIER1+bw)
  Arb TSlave 2 (MM_IMEM row):
    0x0 0x0 0x0 0x0 0x0 0x0 0x0 0x0 0x0 0x0 0x0 0x0 0x0 0x0

Under WiFi DL (Graphics 3D woken up):

  BWSum:
    0x9f4   0x45cb  0x0   0x0      ← SMI = 0x9f4, APPSS = 0x45cb
  Arb TSlave 0 (SMI row):
    0x89f4 0x0 0x0 0x0 0x0      0x0 0x0 0x0 0x0 0x0 0x0 0x0 0x0 0x0
  Arb TSlave 1 (APPSS gateway row):
    0x89f4 0x0 0x0 0x0 0x3bd7   0x0 0x0 0x0 0x0 0x0 0x0 0x0 0x0 0x0
                            ^^^^^^
                            port 4 = GRAPHICS_3D (TIER2+bw, no 0x8000)

Decoding:
  0x3bd7 = (no TIER1 bit) | 0x3bd7
         = 15319 * 128 KB/s = ~1.9 GB/s GPU 3D demand

Master port mapping on MMFAB (mainline enum):
  port 0  = MDP_PORT0
  port 1  = MDP_PORT1
  port 2  = ADM1_PORT0 (MMSS-side ADM master)
  port 3  = ROTATOR
  port 4  = GRAPHICS_3D
  port 5  = JPEG_DEC
  port 6  = GRAPHICS_2D_CORE0
  port 7  = VFE
  port 8  = VPE
  port 9  = JPEG_ENC
  port 10 = GRAPHICS_2D_CORE1
  port 11 = MMFAB_TO_APPSS gateway (the slv_port=1 / mas_port=11 node)
  port 12 = HD_CODEC_PORT0
  port 13 = HD_CODEC_PORT1

Findings:
  - Port 0 (MDP_PORT0) is TIER1 in BOTH TSlave 0 (SMI) and TSlave 1
    (APPSS gateway) rows.  Legacy's per-slave-row arb addressing
    correctly populates BOTH rows because MDP votes BOTH slaves:
      client-data/mdp:
        masters: 22 22
        slaves : 525 512       ← 525 = MMFAB SMI, 512 = APPSS EBI
  - Port 11 (mmfab_to_appss) is ZERO.  Legacy never writes an arb
    cell for the gateway-master role on its parent fabric; only the
    real master cells (MDP, GPU) compete for MMFAB cycles.
  - Port 4 (GRAPHICS_3D) shows up at TIER2 when GPU is active --
    legacy uses TIER2 for GPU traffic.

Implication for mainline parity:
  - mmfab_mas_mdp_port0/1 MUST stay at ARB_TIER1.  Already correct
    in mainline.
  - mmfab_to_appss arb cell on MMFAB MUST be 0 (gateway never
    competes with real masters within its parent fabric).  Mainline
    propagation of MMSS->EBI bw into mmfab_to_appss's mas_port arb
    cell needs to be suppressed via @no_arb.
  - GPU traffic does NOT need TIER1; legacy proves it works at TIER2.

----------------------------------------------------------------------
3. SFAB and DFAB
----------------------------------------------------------------------

  commit-data/msm_sys_fab : 0 lines (no msm_bus_scale consumer)
  commit-data/msm_sys_fpb : 0 lines
  commit-data/msm_cpss_fpb: 0 lines

Legacy SFAB / DFAB / FPB are entirely silent.  SDC and ADM bandwidth
demand is conveyed through the clock tree (dfab_sdc1/2/3/4_clk,
ebi1_adm_clk, dfab_clk, ebi1_clk).  No arb cell is ever programmed on
these fabrics under the vendor SDC/ADM drivers.

Implication for mainline parity:
  - Mainline rpm_commit must allow real ADM master arb cells
    (sfab_mas_adm1_port0/port1) at TIER2 -- the SFAB fabric arbiter
    DOES need to know they exist so it can round-robin properly
    against any other SFAB master.
  - But the path-propagated bw through cross-fabric gateways
    (sfab_to_appss, sfab_to_dfab, dfab_to_sfab) MUST NOT register
    as arb entries.  These nodes already have mas_port=-1 in mainline
    so no fix is required there.

----------------------------------------------------------------------
4. Per-consumer voting summary (client-data)
----------------------------------------------------------------------

  acpuclock   : master 1,  slave 512 (EBI), ib up to 2480 MB/s
  mdp         : master 22, slaves 525+512, ab=334-564 MB/s, ib=417-705
  grp2d0      : master 28, slave 512, ib=2096 MB/s (when active)
  grp2d1      : master 32, slave 512, ib=2096 MB/s (when active)
  grp3d       : master 26, slave 512, ab+ib=2008 MB/s (when active)

No client-data entries for:
  mmc / sdcc1 / sdcc4 / wifi / ath6kl / adm0 / adm1 / qcom_adm
  serial / hsuart / lpass / camera / vfe / vpe

This confirms: legacy SDC, ADM, WiFi, UART, audio, camera all manage
their bandwidth requirements through the clock tree, not through
msm_bus_scale.  The msm_bus arb tables on AFAB / MMFAB therefore see
ONLY the display + GPU subsystems.

----------------------------------------------------------------------
5. Mainline replication plan (this commit)
----------------------------------------------------------------------

(a) Restore afab_to_mmss.mas_tier = ARB_TIER1 (matches legacy).
(b) Add @no_arb flag to struct msm8660_icc_node.
(c) Set @no_arb=true on the two gateways whose mainline-only
    icc_set_bw propagation creates phantom arb cells:
      - afab_to_system (carries SDC/ADM EBI-bound traffic in mainline)
      - mmfab_to_appss (master-side role of the MMFAB->AFAB gateway)
(d) In msm8660_rpm_commit, skip arb writes when @no_arb is set,
    while keeping BWSum aggregation intact so EBI / MMFAB clocks
    still scale with the gateway's accumulated bandwidth demand.

Result: AFAB arb cells match legacy exactly (port 2 = TIER1+bw when
MDP votes, port 3 = 0, all other ports = 0).  MMFAB arb cells also
match (real masters populated per their tier, gateway port = 0).
ADM and SDC concurrent contention should resolve because the only
arb-cell contender on AFAB is the legitimate MMSS->EBI path, leaving
hardware round-robin to serve SFAB-side drains.
