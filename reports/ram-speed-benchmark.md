# HP TouchPad RAM Speed Benchmark

Comparative RAM speed benchmarks between different kernel versions on the HP TouchPad.

## Hardware

- **Device:** HP TouchPad (Topaz)
- **SoC:** Qualcomm APQ8060 (dual-core Scorpion @ 1.5 GHz)
- **RAM:** 1 GB LPDDR2
- **Theoretical Bandwidth:** ~4.2 GB/s (LPDDR2-533, dual-channel)

## Test Methodology

Tests performed using BusyBox `dd` with `time` for measurement:

1. **Memory Bandwidth Test:** `dd if=/dev/zero of=/dev/null bs=1M count=512`
   - Measures raw memory copy throughput (kernel zero-page to null sink)

2. **tmpfs Write:** `dd if=/dev/zero of=/tmp/ramtest bs=1M count=128`
   - Measures write speed to RAM-backed filesystem

3. **tmpfs Read:** `dd if=/tmp/ramtest of=/dev/null bs=1M`
   - Measures read speed from RAM-backed filesystem

---

## Linux 6.18 (LuneOS)

**Test Date:** 2026-01-22
**Kernel:** 6.18.x (mainline port)
**OS:** LuneOS 1.0 (initramfs debug environment)
**Commit:** c86605b9dcfa

### Results

| Test | Size | Time | Speed |
|------|------|------|-------|
| Memory bandwidth (zero→null) | 512 MB | 1.111s | **461 MB/s** |
| tmpfs write | 128 MB | 0.943s | **136 MB/s** |
| tmpfs read | 128 MB | 1.429s | **90 MB/s** |

### Memory Info

```
MemTotal:         983364 kB
MemFree:          929976 kB
MemAvailable:     919732 kB
HighTotal:        251904 kB
LowTotal:         731460 kB
CmaTotal:         262144 kB
```

---

## Linux 2.6.35 (webOS)

**Test Date:** 2026-01-22
**Kernel:** 2.6.35-palm-tenderloin #1 SMP PREEMPT 129.3.13
**OS:** webOS 3.0.5

### Results

| Test | Size | Time | Speed |
|------|------|------|-------|
| Memory bandwidth (zero→null) | 512 MB | 0.555s | **923 MB/s** |
| tmpfs write | 24 MB | 0.429s | **56 MB/s** |
| tmpfs read | 24 MB | 0.188s | **128 MB/s** |

*Note: tmpfs tests used 24 MB due to /var/tmp size limit (32 MB) in webOS. /tmp was full.*

### Memory Info

```
MemTotal:         941544 kB
MemFree:          592016 kB
Buffers:           23156 kB
Cached:           142240 kB
SwapTotal:        524284 kB
SwapFree:         524284 kB
LowTotal:         941544 kB
LowFree:          592016 kB
```

---

## Comparison Summary

| Metric | Linux 6.18 | Linux 2.6.35 | Difference |
|--------|------------|--------------|------------|
| Memory bandwidth | 461 MB/s | 923 MB/s | **-50%** (6.18 slower) |
| tmpfs write | 136 MB/s (128MB) | 56 MB/s (24MB) | *(different test sizes)* |
| tmpfs read | 90 MB/s (128MB) | 128 MB/s (24MB) | *(different test sizes)* |

### Key Findings

1. **Memory bandwidth is 2x slower on Linux 6.18** - This is a significant regression that warrants investigation. Possible causes:
   - Different memory allocator behavior (SLUB vs SLAB)
   - HIGHMEM enabled on 6.18 (251 MB high, 731 MB low) vs flat memory on 2.6.35
   - CMA reservation (256 MB) reducing available contiguous memory
   - Different kernel zero-page or copy optimizations
   - CPU frequency scaling differences

2. **tmpfs tests inconclusive** - Different test sizes make direct comparison invalid. Would need to re-run with identical sizes.

3. **Memory layout differs significantly:**
   - webOS 2.6.35: 941 MB total, flat memory model, 512 MB swap configured
   - Linux 6.18: 983 MB total, HIGHMEM enabled, 256 MB CMA reserved, no swap

---

## Notes

- The `dd` test measures practical throughput including CPU and kernel overhead, not raw hardware bandwidth
- tmpfs performance is lower due to filesystem layer overhead
- Results may vary based on system load and memory pressure
- The 2x memory bandwidth difference suggests kernel memory management optimizations may be needed
- Consider investigating: CONFIG_HIGHMEM, CMA size, memory allocator, ARM copy routines
