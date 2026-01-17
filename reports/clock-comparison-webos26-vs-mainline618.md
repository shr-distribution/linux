# Clock Comparison: webOS 2.6 Kernel vs Mainline Linux 6.18

**Date:** January 17, 2026
**Purpose:** Document clock infrastructure differences between the stock webOS 2.6 kernel and mainline Linux 6.18 for HP TouchPad (APQ8060)

## Executive Summary

Analysis of the clock subsystems reveals that **all critical clocks required for TouchPad operation are present in mainline Linux 6.18**. The clocks are organized differently across multiple clock controller drivers and use modern frameworks (interconnect) instead of legacy bus voting clocks.

## Methodology

1. Connected to HP TouchPad running stock webOS via novacom
2. Mounted debugfs and enumerated all clocks from `/sys/kernel/debug/clk/`
3. Captured enable state and rate for each clock
4. Compared against mainline clock drivers for MSM8660/APQ8060

## Clock Controllers in Mainline Linux 6.18

| Driver | File | Purpose |
|--------|------|---------|
| GCC | `drivers/clk/qcom/gcc-msm8660.c` | GSBI, SDC, USB, ADM, general clocks |
| RPMCC | `drivers/clk/qcom/clk-rpm.c` | Fabric clocks via RPM |
| MMCC | `drivers/clk/qcom/mmcc-msm8960.c` | Display, camera, video clocks |
| LCC | `drivers/clk/qcom/lcc-msm8960.c` | Audio I2S, PCM, slimbus clocks |

### Defconfig Status

All clock controllers are enabled in `arch/arm/configs/tenderloin_defconfig`:

```
CONFIG_COMMON_CLK_QCOM=y
CONFIG_QCOM_CLK_RPM=y
CONFIG_MSM_GCC_8660=y
CONFIG_MSM_LCC_8960=y
CONFIG_MSM_MMCC_8960=y
```

## Enabled Clocks on Running webOS 2.6 System

The following clocks were actively enabled on the stock webOS system:

### Fabric/Bus Clocks (System Infrastructure)
| Clock | Rate | Purpose |
|-------|------|---------|
| afab_clk | 314.5 MHz | Apps Fabric |
| afab_a_clk | 314.5 MHz | Apps Fabric (active) |
| sfab_clk | 157.25 MHz | System Fabric |
| sfab_a_clk | 157.25 MHz | System Fabric (active) |
| dfab_clk | 64 MHz | Daytona Fabric |
| dfab_a_clk | 64 MHz | Daytona Fabric (active) |
| ebi1_clk | 314.5 MHz | External Bus Interface |
| ebi1_a_clk | 314.5 MHz | EBI (active) |

### Peripheral Clocks
| Clock | Rate | Purpose |
|-------|------|---------|
| gsbi10_uart_clk | 64 MHz | Touchscreen UART (4 Mbps) |
| gsbi10_p_clk | - | GSBI10 AHB clock |
| gsbi3_qup_clk | 24 MHz | I2C for sensors |
| gsbi3_p_clk | - | GSBI3 AHB clock |
| gsbi8_qup_clk | 24 MHz | I2C for PMIC/misc |
| gsbi8_p_clk | - | GSBI8 AHB clock |
| sdc1_clk | 48 MHz | eMMC storage |
| sdc1_p_clk | - | SDC1 AHB clock |
| usb_hs1_p_clk | - | USB Host AHB clock |
| adm0_clk | - | DMA engine 0 |
| adm0_p_clk | - | ADM0 AHB clock |

### Audio Clocks
| Clock | Rate | Purpose |
|-------|------|---------|
| codec_i2s_mic_osr_clk | 12.288 MHz | Microphone I2S |
| codec_i2s_mic_bit_clk | 8 | Mic bit clock |
| codec_i2s_spkr_osr_clk | 12.288 MHz | Speaker I2S |
| codec_i2s_spkr_bit_clk | 8 | Speaker bit clock |

### Bus Voting Clocks (Legacy)
| Clock | Rate | Purpose |
|-------|------|---------|
| dfab_sdc1_clk | 64 MHz | Bus vote for SDC1 |
| dfab_usb_hs_clk | MAX | Bus vote for USB HS |
| ebi_adm0_clk | 27 | EBI vote for ADM0 |
| ebi_msmbus_clk | - | MSM Bus EBI vote |

## Clock Name Mapping: webOS 2.6 → Linux 6.18

### Naming Convention Changes

| Pattern in 2.6 | Pattern in 6.18 | Example |
|----------------|-----------------|---------|
| `*_p_clk` | `*_h_clk` | `gsbi10_p_clk` → `gsbi10_h_clk` |
| `*_p_clk` | `*_pbus_clk` | `adm0_p_clk` → `adm0_pbus_clk` |
| `*_p_clk` | `*_ahb_clk` | `mdp_p_clk` → `mdp_ahb_clk` |
| `pixel_lcdc_clk` | `mdp_lcdc_clk` | Display pixel clock |
| `pixel_mdp_clk` | `mdp_pixel_clk` | MDP pixel clock |

### Fabric Clocks Mapping

| webOS 2.6 | Linux 6.18 | Driver |
|-----------|------------|--------|
| afab_clk | RPM_APPS_FABRIC_CLK | clk-rpm.c |
| afab_a_clk | RPM_APPS_FABRIC_A_CLK | clk-rpm.c |
| sfab_clk | RPM_SYS_FABRIC_CLK | clk-rpm.c |
| sfab_a_clk | RPM_SYS_FABRIC_A_CLK | clk-rpm.c |
| mmfab_clk | RPM_MM_FABRIC_CLK | clk-rpm.c |
| mmfab_a_clk | RPM_MM_FABRIC_A_CLK | clk-rpm.c |
| dfab_clk | RPM_DAYTONA_FABRIC_CLK | clk-rpm.c |
| dfab_a_clk | RPM_DAYTONA_FABRIC_A_CLK | clk-rpm.c |
| cfpb_clk | RPM_CFPB_CLK | clk-rpm.c |
| sfpb_clk | RPM_SFPB_CLK | clk-rpm.c |
| mmfpb_clk | RPM_MMFPB_CLK | clk-rpm.c |
| smi_clk | RPM_SMI_CLK | clk-rpm.c |
| ebi1_clk | RPM_EBI1_CLK | clk-rpm.c |

### GCC Clocks (Direct Mapping)

| webOS 2.6 | Linux 6.18 | Notes |
|-----------|------------|-------|
| gsbi*_uart_clk | gsbi*_uart_clk | Same name |
| gsbi*_qup_clk | gsbi*_qup_clk | Same name |
| gsbi*_p_clk | gsbi*_h_clk | Renamed |
| sdc*_clk | sdc*_clk | Same name |
| sdc*_p_clk | sdc*_h_clk | Renamed |
| usb_hs1_xcvr_clk | usb_hs1_xcvr_clk | Same name |
| usb_hs1_p_clk | usb_hs1_h_clk | Renamed |
| adm0_clk | adm0_clk | Same name |
| adm0_p_clk | adm0_pbus_clk | Renamed |

### Audio Clocks (LCC)

| webOS 2.6 | Linux 6.18 | Notes |
|-----------|------------|-------|
| codec_i2s_mic_osr_clk | codec_i2s_mic_osr_clk | Same name |
| codec_i2s_mic_bit_clk | codec_i2s_mic_bit_clk | Same name |
| codec_i2s_spkr_osr_clk | codec_i2s_spkr_osr_clk | Same name |
| codec_i2s_spkr_bit_clk | codec_i2s_spkr_bit_clk | Same name |
| spare_i2s_*_clk | spare_i2s_*_clk | Same name |
| mi2s_osr_clk | mi2s_osr_clk | Same name |
| mi2s_bit_clk | mi2s_bit_clk | Same name |
| pcm_clk | pcm_clk | Same name |

### Display/Media Clocks (MMCC)

| webOS 2.6 | Linux 6.18 | Notes |
|-----------|------------|-------|
| mdp_clk | mdp_clk | Same name |
| mdp_axi_clk | mdp_axi_clk | Same name |
| mdp_p_clk | mdp_ahb_clk | Renamed |
| mdp_vsync_clk | mdp_vsync_clk | Same name |
| pixel_lcdc_clk | mdp_lcdc_clk | Renamed |
| pixel_mdp_clk | mdp_pixel_clk | Renamed |
| rot_clk | rot_clk | Same name |
| rot_axi_clk | rot_axi_clk | Same name |
| gfx2d0_clk | gfx2d0_clk | Same name |
| gfx3d_clk | gfx3d_clk | Same name |
| vfe_clk | vfe_clk | Same name |
| vpe_clk | vpe_clk | Same name |
| vcodec_clk | vcodec_clk | Same name |

## Bus Voting: Legacy vs Interconnect Framework

### Legacy Approach (webOS 2.6)

The 2.6 kernel used per-peripheral bus voting clocks:
- `dfab_sdc1_clk` - Bus bandwidth vote for SDC1
- `dfab_sdc2_clk` - Bus bandwidth vote for SDC2
- `dfab_usb_hs_clk` - Bus bandwidth vote for USB HS
- `ebi_adm0_clk` - EBI vote for ADM0 DMA
- `ebi_msmbus_clk` - General MSM bus EBI vote

### Modern Approach (Linux 6.18)

The interconnect framework (`drivers/interconnect/qcom/msm8660.c`) replaces individual bus voting clocks:

```
Device Tree Structure:
├── apps_fabric (RPM_APPS_FABRIC_CLK)
├── system_fabric (RPM_SYS_FABRIC_CLK)
└── mmss_fabric (RPM_MM_FABRIC_CLK)
```

Peripherals use `interconnects` property in device tree to specify bandwidth requirements, which the framework translates to appropriate fabric clock rates.

## Clocks Not Found in Mainline

The following clocks from webOS 2.6 do not have direct equivalents in mainline:

| Clock | Purpose | Impact |
|-------|---------|--------|
| usb_phy0_clk | USB PHY clock | USB works, handled differently |
| pdm_clk | Pulse Density Modulation | Not used on TouchPad |
| tssc_clk | Touch Screen Sample Clock | Not needed (Cypress uses UART) |
| ppss_p_clk | Peripheral Sensor Subsystem | Not used |
| ce2_p_clk | Crypto Engine 2 | Not critical |
| dfab_sdc*_clk | Per-device bus votes | Handled by interconnect |
| dfab_usb_hs_clk | USB bus vote | Handled by interconnect |
| ebi_adm*_clk | DMA EBI votes | Handled by interconnect |
| ebi_msmbus_clk | Bus EBI vote | Handled by interconnect |

**None of these missing clocks should affect TouchPad functionality.**

## Complete Clock List from webOS 2.6

For reference, here is the complete list of 171 clocks enumerated from the running webOS system:

```
adm0_clk              adm0_p_clk            adm1_clk              adm1_p_clk
afab_a_clk            afab_clk              amp_clk               amp_p_clk
cam_clk               ce2_p_clk             cfpb_a_clk            cfpb_clk
codec_i2s_mic_bit_clk codec_i2s_mic_osr_clk codec_i2s_spkr_bit_clk codec_i2s_spkr_osr_clk
csi0_clk              csi0_p_clk            csi0_vfe_clk          csi1_clk
csi1_p_clk            csi1_vfe_clk          csi_src_clk           dfab_a_clk
dfab_clk              dfab_dsps_clk         dfab_sdc1_clk         dfab_sdc2_clk
dfab_sdc3_clk         dfab_sdc4_clk         dfab_sdc5_clk         dfab_usb_hs_clk
dsi_byte_clk          dsi_esc_clk           dsi_m_p_clk           dsi_s_p_clk
ebi1_a_clk            ebi1_clk              ebi_adm0_clk          ebi_adm1_clk
ebi_msmbus_clk        gfx2d0_clk            gfx2d0_p_clk          gfx2d1_clk
gfx2d1_p_clk          gfx3d_clk             gfx3d_p_clk           gsbi10_p_clk
gsbi10_qup_clk        gsbi10_uart_clk       gsbi11_p_clk          gsbi11_qup_clk
gsbi11_uart_clk       gsbi12_p_clk          gsbi12_qup_clk        gsbi12_uart_clk
gsbi1_p_clk           gsbi1_qup_clk         gsbi1_uart_clk        gsbi2_p_clk
gsbi2_qup_clk         gsbi2_uart_clk        gsbi3_p_clk           gsbi3_qup_clk
gsbi3_uart_clk        gsbi4_p_clk           gsbi4_qup_clk         gsbi4_uart_clk
gsbi5_p_clk           gsbi5_qup_clk         gsbi5_uart_clk        gsbi6_p_clk
gsbi6_qup_clk         gsbi6_uart_clk        gsbi7_p_clk           gsbi7_qup_clk
gsbi7_uart_clk        gsbi8_p_clk           gsbi8_qup_clk         gsbi8_uart_clk
gsbi9_p_clk           gsbi9_qup_clk         gsbi9_uart_clk        hdmi_app_clk
hdmi_m_p_clk          hdmi_s_p_clk          hdmi_tv_clk           ijpeg_axi_clk
ijpeg_clk             ijpeg_p_clk           imem_axi_clk          imem_p_clk
jpegd_axi_clk         jpegd_clk             jpegd_p_clk           mdp_axi_clk
mdp_clk               mdp_p_clk             mdp_tv_clk            mdp_vsync_clk
mi2s_bit_clk          mi2s_osr_clk          mmfab_a_clk           mmfab_clk
mmfpb_a_clk           mmfpb_clk             modem_ahb1_p_clk      modem_ahb2_p_clk
pcm_clk               pdm_clk               pixel_lcdc_clk        pixel_mdp_clk
pmem_clk              pmic_arb0_p_clk       pmic_arb1_p_clk       pmic_ssbi2_clk
ppss_p_clk            prng_clk              rot_axi_clk           rot_clk
rot_p_clk             rpm_msg_ram_p_clk     sdc1_clk              sdc1_p_clk
sdc2_clk              sdc2_p_clk            sdc3_clk              sdc3_p_clk
sdc4_clk              sdc4_p_clk            sdc5_clk              sdc5_p_clk
sfab_a_clk            sfab_clk              sfpb_a_clk            sfpb_clk
smi_a_clk             smi_clk               smmu_p_clk            spare_i2s_mic_bit_clk
spare_i2s_mic_osr_clk spare_i2s_spkr_bit_clk spare_i2s_spkr_osr_clk tsif_p_clk
tsif_ref_clk          tssc_clk              tv_dac_clk            tv_enc_clk
tv_enc_p_clk          tv_src_clk            usb_fs1_p_clk         usb_fs1_src_clk
usb_fs1_sys_clk       usb_fs1_xcvr_clk      usb_fs2_p_clk         usb_fs2_src_clk
usb_fs2_sys_clk       usb_fs2_xcvr_clk      usb_hs1_p_clk         usb_hs1_xcvr_clk
usb_phy0_clk          vcodec_axi_clk        vcodec_clk            vcodec_p_clk
vfe_axi_clk           vfe_clk               vfe_p_clk             vpe_axi_clk
vpe_clk               vpe_p_clk
```

## Conclusions

1. **Clock infrastructure is complete** - All critical clocks for TouchPad operation exist in mainline Linux 6.18

2. **Naming differences are cosmetic** - The `_p_clk` → `_h_clk` renaming follows upstream conventions

3. **Bus voting modernized** - Legacy per-device bus clocks replaced by interconnect framework

4. **Audio clocks present** - Full I2S support via LCC driver

5. **Display clocks present** - MDP, pixel, and LCDC clocks available via MMCC

6. **No blocking issues** - Missing clocks (pdm, tssc, etc.) are not needed for TouchPad functionality

## Recommendations

1. Verify RPM communication is working correctly (fabric clocks depend on it)
2. Ensure device tree nodes properly reference clock controllers
3. Check interconnect paths are configured for peripherals needing bus bandwidth
4. If debugging clock issues, compare actual rates against this reference

## References

- webOS 2.6 kernel source (downstream Qualcomm)
- Linux 6.18 `drivers/clk/qcom/` clock drivers
- Linux 6.18 `drivers/interconnect/qcom/msm8660.c`
- Device tree: `arch/arm/boot/dts/qcom/qcom-msm8660.dtsi`
