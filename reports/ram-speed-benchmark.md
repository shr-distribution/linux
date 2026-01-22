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
**CPU:** 2x Scorpion @ 1512 MHz, governor: performance

### Results

| Test | Size | Time | Speed |
|------|------|------|-------|
| Memory bandwidth (zero→null) | 512 MB | 1.110s (avg of 5) | **461 MB/s** |
| tmpfs write | 24 MB | 0.575s (avg of 3) | **42 MB/s** |
| tmpfs read | 24 MB | 0.222s (avg of 3) | **108 MB/s** |

*Note: Multiple runs showed consistent ~1.1s for memory bandwidth. tmpfs read varies due to cache effects.*

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

## Comparison Summary (Fair Comparison - Same Parameters)

| Metric | Linux 6.18 | Linux 2.6.35 | Difference |
|--------|------------|--------------|------------|
| Memory bandwidth (512 MB) | 461 MB/s | 923 MB/s | **-50%** (6.18 slower) |
| tmpfs write (24 MB) | 42 MB/s | 56 MB/s | **-25%** (6.18 slower) |
| tmpfs read (24 MB) | 108 MB/s | 128 MB/s | **-16%** (6.18 slower) |

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

## Kernel Configuration Comparison

Detailed comparison of `reference/webos-2.6.35-kernel-config` vs `arch/arm/configs/tenderloin_debug_defconfig`:

### Timer/Scheduler (HIGH IMPACT)

| Option | webOS 2.6.35 | Linux 6.18 | Impact |
|--------|--------------|------------|--------|
| CONFIG_HZ | **100** | **1000** | 10x more timer interrupts on 6.18 |
| CONFIG_PREEMPT | y | y | Same |
| CONFIG_NO_HZ | y | y | Same |
| CONFIG_HIGH_RES_TIMERS | y | y | Same |

**Analysis:** HZ=1000 means 10x more timer interrupts per second. Each interrupt requires context switching overhead which can significantly impact memory-intensive benchmarks like `dd`. This is likely a **major contributor** to the performance difference.

### Memory Management (HIGH IMPACT)

| Option | webOS 2.6.35 | Linux 6.18 | Impact |
|--------|--------------|------------|--------|
| CONFIG_VMSPLIT | **2G/2G** | 3G/1G (default) | Different VM layout |
| CONFIG_CMA | n/a | **256 MB** | Large reservation on 6.18 |
| CONFIG_MEMCG | n | **y** | Cgroups memory tracking overhead |
| CONFIG_KSM | n | **y** | Kernel samepage merging overhead |
| CONFIG_HIGHMEM | y | y | Same (but different amounts) |
| CONFIG_FLATMEM | y | (default) | Same |
| CONFIG_SLUB | y | y | Same allocator |
| CONFIG_BOUNCE | **y** | n | Bounce buffers on webOS |
| CONFIG_SWAP | **y** | n | Swap enabled on webOS |

**Analysis:**
- VMSPLIT_2G gives webOS equal kernel/user space, potentially better for memory operations
- CMA reserves 256MB on 6.18, fragmenting available memory
- MEMCG adds overhead for every allocation to track cgroup membership
- KSM background scanning adds CPU overhead

### ARM-Specific (MEDIUM IMPACT)

| Option | webOS 2.6.35 | Linux 6.18 | Impact |
|--------|--------------|------------|--------|
| CONFIG_ARM_DMA_MEM_BUFFERABLE | **y** | n | Write-combining for DMA on webOS |
| CONFIG_OABI_COMPAT | y | n | Old ABI compat on webOS |
| CONFIG_ARM_THUMB | y | n | Thumb instructions on webOS |
| VFPv3 | explicit | implicit | Same FPU |
| NEON | y | y | Same SIMD |

**Analysis:** ARM_DMA_MEM_BUFFERABLE enables write-combining for DMA memory regions, which can significantly improve memory throughput for certain operations.

### Debug/Tracing (MEDIUM IMPACT)

| Option | webOS 2.6.35 | Linux 6.18 | Impact |
|--------|--------------|------------|--------|
| CONFIG_FUNCTION_TRACER | n | **y** | Function call tracing overhead |
| CONFIG_FTRACE_SYSCALLS | n | **y** | Syscall tracing overhead |
| CONFIG_DYNAMIC_DEBUG | n | **y** | Dynamic debug infrastructure |
| CONFIG_SLUB_DEBUG | **y** | n | SLUB debugging (disabled on 6.18 - good) |
| CONFIG_DEBUG_INFO_DWARF4 | n | **y** | Larger kernel image |

**Analysis:** Function tracing adds overhead to every function call. While not enormous, it accumulates in tight loops like memory copies.

### Compiler Optimization

| Option | webOS 2.6.35 | Linux 6.18 | Impact |
|--------|--------------|------------|--------|
| CONFIG_CC_OPTIMIZE_FOR_SIZE | **y** | n (speed default) | Smaller but slower code on webOS? |

**Analysis:** Surprisingly, webOS uses -Os (optimize for size) which typically produces slower code than -O2. This should favor 6.18, yet 6.18 is slower. This suggests the other factors outweigh compiler optimization.

---

## Recommendations

Based on the configuration analysis, try these changes to improve 6.18 performance:

### High Priority

1. **Reduce HZ to 100 or 250:**
   ```
   CONFIG_HZ_100=y  (or CONFIG_HZ_250=y)
   # CONFIG_HZ_1000 is not set
   ```

2. **Reduce CMA reservation:**
   ```
   CONFIG_CMA_SIZE_MBYTES=64  (or 128, down from 256)
   ```

3. **Add VMSPLIT_2G:**
   ```
   CONFIG_VMSPLIT_2G=y
   ```

### Medium Priority

4. **Disable cgroups memory controller (if not needed):**
   ```
   # CONFIG_MEMCG is not set
   ```

5. **Disable KSM (if not needed):**
   ```
   # CONFIG_KSM is not set
   ```

6. **Disable function tracing for production:**
   ```
   # CONFIG_FUNCTION_TRACER is not set
   # CONFIG_FTRACE_SYSCALLS is not set
   ```

### Experimental

7. **Enable ARM DMA bufferable (needs testing):**
   ```
   CONFIG_ARM_DMA_MEM_BUFFERABLE=y
   ```

---

## Notes

- The `dd` test measures practical throughput including CPU and kernel overhead, not raw hardware bandwidth
- tmpfs performance is lower due to filesystem layer overhead
- Results may vary based on system load and memory pressure
- The 2x memory bandwidth difference is likely due to cumulative effects of HZ, CMA, MEMCG, and tracing
- A `tenderloin_fast_defconfig` could be created with these optimizations for performance testing
