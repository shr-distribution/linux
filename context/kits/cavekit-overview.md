---
created: "2026-05-11"
last_edited: "2026-05-11"
---

# Cavekit Overview

## Project
linux-6.18-tenderloin — Linux 6.18 kernel port for the HP TouchPad (Qualcomm APQ8060, LuneOS).

## Domain Index
| Domain | File | Summary | Status |
|--------|------|---------|--------|
| spm-init | cavekit-spm-init.md | Initialize SPM/SAW registers at probe for MSM8660 power collapse | draft |

## Cross-Reference Map
| Domain A | Interacts With | Interaction Type |
|----------|---------------|-----------------|
| spm-init | cpuidle | Enables | Provides register init for cpu-spc state |
| spm-init | cpu-hotplug | Enables | Provides register init for hotplug power collapse |

## Dependency Graph
```
spm-init (foundation)
  ├─> cpu-hotplug (already implemented)
  └─> cpuidle-spc (pending RPM orchestrator)
```
