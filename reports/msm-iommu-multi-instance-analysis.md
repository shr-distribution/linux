# MSM IOMMU Multi-Instance Support Analysis

## Problem Statement

The HP TouchPad's MDP4 display controller has two DMA ports, each with its own
IOMMU instance:
- `mdp_port0_iommu` @ 0x07500000 - for VG1/RGB1 pipes
- `mdp_port1_iommu` @ 0x07600000 - for VG2/RGB2 pipes

The mainline `msm_iommu` driver had a bug in `insert_iommu_master()` that caused
incorrect behavior when a device references multiple IOMMU instances.

## Root Cause

In `drivers/iommu/msm_iommu.c`, the original `insert_iommu_master()` function
used `dev_iommu_priv_get(dev)` to store a single master context pointer:

```c
static int insert_iommu_master(struct device *dev,
                               struct msm_iommu_dev **iommu,
                               const struct of_phandle_args *spec)
{
    struct msm_iommu_ctx_dev *master = dev_iommu_priv_get(dev);
    ...
    if (list_empty(&(*iommu)->ctx_list)) {
        master = kzalloc(sizeof(*master), GFP_ATOMIC);
        ...
        dev_iommu_priv_set(dev, master);  // Overwrites on each IOMMU!
    }
    master->mids[master->num_mids++] = spec->args[0];  // Wrong master!
    ...
}
```

The bug: When a device like MDP references multiple IOMMU instances:
1. First call (port0): Creates master1, adds to port0's ctx_list, sets dev priv
2. Second call (port1): Creates master2, adds to port1's ctx_list, OVERWRITES dev priv
3. Subsequent calls for port0: Gets master2 (wrong!), adds MIDs to wrong master

This caused MIDs to be programmed on the wrong IOMMU instance.

## Solution Implemented

The fix adds a `find_master_for_dev()` helper that looks up the correct master
context for each IOMMU instance, rather than relying on `dev_iommu_priv_get()`:

```c
static struct msm_iommu_ctx_dev *find_master_for_dev(struct msm_iommu_dev *iommu,
                                                     struct device *dev)
{
    struct msm_iommu_ctx_dev *master;

    list_for_each_entry(master, &iommu->ctx_list, list) {
        if (master->of_node == dev->of_node)
            return master;
    }
    return NULL;
}
```

This function is now used in:
- `insert_iommu_master()` - to find/create the correct master per IOMMU
- `find_iommu_for_dev()` - to locate IOMMUs for a device
- `msm_iommu_attach_dev()` - to attach only the device's master on each IOMMU
- `msm_iommu_identity_attach()` - to detach only the device's master

## Driver Architecture

The msm_iommu driver already had multi-instance support through:

1. `struct msm_priv::list_attached` - list of attached IOMMU devices per domain
2. TLB flush operations iterate over all attached IOMMUs
3. Each IOMMU's `ctx_list` contains master contexts for devices using it

The fix ensures each device gets a separate master context on each IOMMU it
references, and operations correctly target only that device's masters.

## Files Modified

- `drivers/iommu/msm_iommu.c` - Multi-instance bug fix
- `arch/arm/configs/tenderloin_defconfig` - Enable CONFIG_MSM_IOMMU
- `arch/arm/configs/tenderloin_debug_defconfig` - Enable CONFIG_MSM_IOMMU
- `arch/arm/configs/tenderloin_fast_defconfig` - Enable CONFIG_MSM_IOMMU

## Device Tree Configuration

MDP IOMMU references in `qcom-apq8060-tenderloin-common.dtsi`:
```dts
mdp: display-controller@5100000 {
    iommus = <&mdp_port0_iommu 0>,   /* VG1 ctx */
             <&mdp_port0_iommu 1>,   /* RGB1 ctx */
             ...
             <&mdp_port1_iommu 0>,   /* VG2 ctx */
             <&mdp_port1_iommu 1>,   /* RGB2 ctx */
             ...;
};
```

## Testing Notes

After enabling IOMMU:
1. Build kernel with CONFIG_MSM_IOMMU=y
2. Uncomment the iommus property in the MDP device tree node
3. Test display output and GPU operations
4. Check dmesg for IOMMU fault messages

## References

- Legacy kernel: `arch/arm/mach-msm/iommu.c`
- Legacy device definitions: `arch/arm/mach-msm/devices-msm8x60-iommu.c`
- MDP contexts: mdp_vg1_ctx, mdp_rgb1_ctx (port0), mdp_vg2_ctx, mdp_rgb2_ctx (port1)
