# Upstream Review Status

Companion to [STAGES.md](STAGES.md). STAGES.md is the static roadmap (what
each series does and when it ships); this document is the **live status**
(where each branch sits in send/review right now, and what the most recent
Sashiko/maintainer findings were).

Update **the row** when a branch's state changes. Append new findings to
the **Findings Log** at the bottom. Bump the "Last updated" stamp every
time you change either.

**Last updated:** 2026-06-07 18:00 CEST

---

## Tooling status

| Component | State | Note |
|---|---|---|
| `claude-cli` (Opus 4.7) | working | Used for ad-hoc reviews when rate window is fresh |
| `claude-cli` (Sonnet 4.6) | working but **slow on 3000+ line drivers** | Two mt9m113 v8 escalation runs on 2026-06-07 both timed out — once at 60min, once at 2h wrapper cap. Use Haiku or split work for big drivers |
| Vertex (Gemini 2.5 Pro / 3.1 Pro Preview) | **BLOCKED — Google account `qcomapq8060@gmail.com` restricted 2026-06-07** | Sashiko fan-spawn burst on v8 escalation tripped Google abuse-detection. `project-1e644f1f-...` got CONSUMER_SUSPENDED first; then the parent account got `access_denied: Account Restricted`. Recovery: user re-auth flow at accounts.google.com/info/servicerestricted. See [[feedback-vertex-burst-suspension]]. Other project (`project-fbffa5c0-...`) + account (`denisedcruz42@`) are inactive backups at the same risk |
| Sashiko reliability fixes (this session) | **shipped to `~/.cargo/bin/`** | 5 fixes committed to local `~/webos/sashiko` main as `b49af62`: vertex.rs global-host hostname; prompts.rs Fatal short-circuit + RateLimit retry-after sleep + markdown-fence stripping; sashiko-cli stderr-dump on stage-failure exit. Not pushed to sashiko-dev upstream — held local |
| `sashiko-preflight.sh` | working | `--repo` points at integration tree; wrapper timeout = 7200s; diagnostic strings on lines 221/234 correctly say "2h" not "60 min" |
| `pre-send-check.sh` / `send-series.sh` | working | sparse-strict + kernel cocci + smatch + clang-analyzer + .config-built-check; mandatory before `git send-email` |
| Device 172.16.42.2:22 | **tenderloin/linux-next with mt9m113 v7 + KFENCE on-device verified** | KASAN added to tenderloin_debug_defconfig; v7 brownout / race / stream sweeps clean |

---

## Stage 1 — clocks / interconnect / wake / thermal

Dependency notation: `→ #N` means **this row must wait for row N to land**.
Rows are ordered: independent first, then dependents grouped at the end.

### Independent (no cross-series deps)

| # | Branch | Commits | Deps | Sent | Last Sashiko | Pending |
|---|---|---|---|---|---|---|
| 0 | `clk-gdsc-msm8x60-legacy` (Set A v4) | 5 | — | v1 `20260602050840.435933-1` | v4 verify clean (Set A self-incurred fixes + recovered LEGACY_FOOTSWITCH + RPM_ALWAYS_ON content #178); Stage 1 r3+r4 chain clean | v2 reroll ready; wait for lkml v1 feedback |
| 0b | `clk-gdsc-preexisting-fixes` (Set B) | 3 | — | **v1 sent 2026-06-02 16:09** `20260602140934.796697-1` | 2026-06-02 15:52 Vertex r1 ✅ 4 findings, all `preexisting:true` | Wait for maintainer review |
| 1 | `dt-bindings-gcc-msm8660-cleanup` | 3 | — | v1 `20260602042747.277270-1` (2 patches); v2 staged (strict YAML pll4 + halt-bit comment + clk-pll NULL-check separate) | claude-cli r1 17:58 ✅: 2 findings (false-positive); CE2 alias revert + GFX2D resets folded | Hold v2 send pending lkml v1 feedback (#149) |
| 3a | `clk-lcc-msm8960-hardening` | 3 | — | **v1 sent 2026-06-02** `20260602045002.290918-1` | v5-r2 clean; v6 added `.suppress_bind_attrs` per Sashiko v5 | Wait for review |
| 4 | `interconnect-msm8660` | 3 | — | not sent | v5-r2 clean (drop arb_size+1 vestigial alloc); v4 fixed double clk cleanup + redundant device_link_remove + propagate clk_set_rate err + fix OOB | **Pending send (#141)** — Stage 1 last send chain |
| 5 | `irqchip-msm8660-mpm` | 2 | — | not sent | v5 clean (threaded IRQ + bus_lock + Kconfig cleanup from v4 findings); USB regression diagnosed + on-device verified | **Pending send (#142)** |
| 6 | `thermal-pm8901-tm` | 3 | — | not sent | v5-r2 clean (preserve OVRD bits in ISR blind-clear High); v4 propagated suspend err + probe rollback + ISR fixes (stage bits + temp read) | **Pending send (#143)** |
| side | `clk-qcom-pll-vote-null-check` | 1 | — | **sent standalone 2026-06-02** | clean (split from gcc-cleanup v2) | Wait for review |

### Dependent (held until prereq lands in -next)

| # | Branch | Commits | Deps | Sent | Last Sashiko | Pending |
|---|---|---|---|---|---|---|
| 2 | `clk-mmcc-msm8660` | 3 | **→ #0** (gdsc framework provides LEGACY_FOOTSWITCH + RPM_ALWAYS_ON) | v1 `20260602043623.285901-1` | r7 clean (596 KB log); v2 fold: 5 Sashiko findings + GFX2D[01]_AHB_RESET entries + unhalt_fabric_ports -EPROBE_DEFER | v2 reroll drafted; blocked on #0 v2 land |
| 3b | `clk-lcc-msm8660` | 2 | **→ #1** (PLL4_VOTE source in gcc-msm8660) | held | Stage 1 r3 clean (lcc-add fix) + LCC merge v3-r2 minimal (drop singleton/resume quirks per Sashiko v1) + bit_clk mux shift 18→14 for MSM8x60 (#124) | Unblocked once #1 v2 lands |
| 6b | `arm-dts-qcom-msm8660-gcc-pll4` | 2 | **→ #1 v2** (strict binding requires pll4); **→ #3b** (`qcom,lcc-msm8660` driver compatible needed for `&lcc` node bind) | not sent | not yet Sashiko-reviewed (DTS-only) | Both deps land → send |

## Stage 2 — charger / phy / crypto / media VFE fix

All independent (no cross-series deps).

| # | Branch | Commits | Deps | Sent | Last Sashiko | Pending |
|---|---|---|---|---|---|---|
| 7 | `media-camss-vfe-17x-wm-done-fix` | 2 | — | not sent | v3 ready; Stage 2 v5 fixes folded; full 9-check sweep clean (#176) | Send |
| 8 | `phy-usb-hs-vendor-init-seq` | 1 | — | not sent | v5 clean (uint8-array schema); v2 addressed 3 preexisting findings (#112); v5 preflight chain clean (#117) | Send |
| 9 | `power-max8903-dc-limit` | 2 | — | not sent | v5 clean (source_lock + DT gpio_value=0 require); v2 fixed 5 NEW Sashiko findings (#111) | Send |
| 10 | `crypto-qce-msm8660` | 2 | — | not sent | v5 ready (cpu_to_be32; arch deferrals documented); **r2 pending: 3 Criticals + 4 High remain (#114)** | Address r2 then send |

## Stage 3 — standalone new drivers

### Independent (no cross-series deps)

| # | Branch | Commits | Deps | Sent | Last Sashiko | Pending |
|---|---|---|---|---|---|---|
| 11 | `media-mt9m113` | 3 | — | **v7 sent** (after v6 sent `20260603040026...`; bumped to v7 with 3 Sashiko-missed + 2 clang fixes — #171) | **v8 prep held local**: 8 driver + 1 binding Sashiko gemini-3.1 findings folded (#179). **Sonnet escalation in flight (b1yqrbht0)**; Gemini-3.1-pro queued after | Decide on v8 send after Sonnet/Gemini-3.1 verdict + lkml v7 feedback |
| 12 | `input-cy8ctma395` | 3 | — | not sent | r5 v2 fixes folded (Critical + 7 High, #151); r3 triage clean (#148); fuzz fix live + on-device verified | Send after Stage 1 lands |
| 13 | `power-supply-palm-a6` | 2 | — | not sent | r1 done + 3 sparse fixes (sprintf overflow + 2 unused-but-set, #167); full pre-send sweep clean (#166) | A6 5-way split in progress (#131); send after split |
| 14 | `dt-bindings-mfd-lm8502` | 1 | — | not sent | r1 done | Review and send (no dep) |
| 15 | `mfd-lm8502` | 3 | — | **v2 sent 2026-06-03 06:00** `20260603040026.398009-1` (v1 sent without cover letter — v2 resend with apology) | Vertex r2 ✅ 3 findings: 2 false + 1 SPDX style (fixed); LM8502 r2 (mfd + haptic) confirmation pending (#128) | Wait for maintainer review |

### Dependent (LM8502 MFD child drivers)

| # | Branch | Commits | Deps | Sent | Last Sashiko | Pending |
|---|---|---|---|---|---|---|
| 16 | `leds-lm8502` | 1 | **→ #14** (binding) + **→ #15** (mfd core) | not sent | r1 done | Send after #14/#15 land |
| 17 | `input-lm8502-haptic` | 1 | **→ #14** (binding) + **→ #15** (mfd core) | not sent | r5 clean (D10 race fix landed in r4 #134); r5 triage clean (#137) | Send after #14/#15 land |

## Stage 4 — MSM8x60 multimedia stack (NEW)

Send after Stage 1 (clocks, interconnect) stabilises in -next. None of these
have been Sashiko-reviewed in the current cycle — they were created during
the Stage 3 expansion (#126) but missed by REVIEW-STATUS.md. **Need fresh
Sashiko sweep before send.**

| # | Branch | Commits | Deps | Sent | Last Sashiko | Pending |
|---|---|---|---|---|---|---|
| 18 | `media-camss-msm8660` (VFE 3.1 / CSI 8x60 core) | 2 | **→ #7** (vfe-17x wm-done-fix in -next) | not sent | **never Sashiko-reviewed**; full 9-check sweep clean (#176, #177) | Sashiko sweep → send after Stage 2 #7 lands |
| 19 | `media-msm8660-rotator` | 3 | — | not sent | **never Sashiko-reviewed**; pre-send sweep clean (#166); includes MAX_BURST_SIZE=0x42 fix | Sashiko sweep → send |
| 20 | `media-msm8660-vpe` | 2 | — | not sent | vpe-r3 triage done (#144); vpe-r2 3 findings fixed (#136); pre-send sweep clean (#166) | Final Sashiko sweep → send |
| 21 | `media-msm8660-vidc` | 2 | — | not sent | **vidc cond_no_effect fix landed (#177)**; was CONFIG-disabled (false-PASS) in earlier sweep — re-scan after fix clean | Sashiko sweep → send |

## Stage 5 — Tenderloin sensor enablement (NEW)

Independent IIO patches needed for the TouchPad's sensor stack (sensorfw via
iio-sensors-adaptor). Not in STAGES.md yet — these were added during the
on-device sensor work but missed by REVIEW-STATUS.md. None reviewed by
Sashiko or sent.

| # | Branch | Commits | Deps | Sent | Last Sashiko | Pending |
|---|---|---|---|---|---|---|
| 22 | `iio-isl29018-cover-comp` | 3 | — | not sent | **never Sashiko-reviewed** | Includes overflow + precision fix to `isl29018_read_lux()` (standalone bugfix) + DT binding for `isil,cover-comp-gain` + driver support. Sashiko sweep → send (bugfix-only patch could go first as a fix-only) |
| 23 | `iio-lsm303dlh-magn-fixes` | 3 | — | not sent | **never Sashiko-reviewed** | st_magn `st,fullscale-mg` DT property + binding + endianness fix in `read_axis_data`. Memory note: [[lsm303dlh-be-and-bias]] — magn outputs BE at 0x03/0x05/0x07; tenderloin needs ≥±2.5G to avoid X saturation. Sashiko sweep → send |
| 24 | `iio-mpu3050-fifo-raw-read` | 1 | — | not sent | **never Sashiko-reviewed** | Read gyro samples via FIFO in `IIO_CHAN_INFO_RAW`. Sashiko sweep → send |
| 25 | `iio-mpu3050-pm-resume-restore-state` | 1 | — | not sent | **never Sashiko-reviewed** | Restore cached sample rate on runtime resume. Sashiko sweep → send |

---

## In-flight

_(Things actively running right now. Move to a Stage row when done.)_

- _(none — mt9m113 v8 escalation closed 2026-06-07 17:50 after seven attempts; see Findings Log)_

---

## Maintainer reply threads sent

| Date | Series | Thread / msg-id | Subject |
|---|---|---|---|
| 2026-06-02 | mmcc-msm8660 | (in reply to `20260602043623.285901-1`) | Confirm Critical mutex_init race fix |
| 2026-06-02 | mmcc-msm8660 | (in reply to `20260602043623.285901-1`) | F9.b fabric-port + GFX2D resets |
| 2026-06-02 | mmcc-msm8660 | (in reply to `20260602043623.285901-1`) | unhalt fabric ports -EPROBE_DEFER cross-check |
| 2026-06-02 | gcc-cleanup | (in reply to `20260602042747.277270-1`) | YAML clock-names third entry |
| 2026-06-02 | gcc-cleanup | (in reply to `20260602042747.277270-1`) | clk-pll vote NULL-check follow-up |

---

## Findings Log

_(Most recent first. Each entry: date / branch / round / model / outcome.
For non-empty findings include the count by severity and a one-line summary
of the most actionable one.)_

### 2026-06-07 17:50 — mt9m113 v8 escalation review CLOSED after 7 attempts — partial clean verdict, Google account restricted

Seven escalation attempts to second-opinion-review the v8 driver, all blocked by infrastructure (not driver issues):

| # | Provider/Model | Outcome |
|---|---|---|
| 1 | claude-sonnet-4-6 | TIMED OUT 60min (initial wrapper cap) |
| 2 | claude-sonnet-4-6 | TIMED OUT 2h (raised cap) — model too slow for 3000-line driver |
| 3 | Vertex gemini-3.1-pro-preview, us-central1 | 404 model-not-found; routing investigation found `vertex.rs` was emitting bogus host `global-aiplatform.googleapis.com`. Patched. |
| 4 | Vertex gemini-3.1-pro-preview, global | rc=101 panic "vertex provider requires the 'vertex' feature" — Cargo features default to `bedrock` only; rebuilt with `--features vertex` |
| 5 | Vertex gemini-3.1-pro-preview, global | All stages 429 rate-limited (preview-model quota); `review` worker fan-spawns 7 stages × 3 inner × 3 outer = burst |
| 6 | Vertex gemini-2.5-pro, project-1e644f1f-... | 429 spread → 403 `CONSUMER_SUSPENDED` mid-run. Switched to project-fbffa5c0-... |
| 7 | Vertex gemini-2.5-pro, project-fbffa5c0-... | Stage 1 returned `{"concerns":[],"dismissed_concerns":[]}` = **clean partial verdict**. Then `access_denied: Account Restricted` killed all subsequent stages |

**Result for mt9m113 v8:** held local. v7 already on lkml; v8 has 8 driver + 1 binding gemini-3.1 findings already folded (earlier round); on-device sweeps clean; gemini-2.5-pro found nothing in the one stage that ran before account restriction. Send when maintainer v7 feedback arrives.

**Sashiko fixes shipped** during the debug saga (committed `b49af62` on local `~/webos/sashiko` main, installed in `~/.cargo/bin/`):
1. `ai/vertex.rs` global-region hostname (drop `global-` prefix)
2. `worker/prompts.rs` short-circuit Fatal upstream ClaudeError before retry
3. `worker/prompts.rs` honor RateLimit retry-after (sleep before next inner attempt)
4. `worker/prompts.rs` strip markdown ```...``` fences before strict JSON parse
5. `bin/sashiko-cli.rs` dump captured review stderr on stage-failure exit so root cause is visible

**Saved memory entries:** [[feedback-sashiko-rebuild-install]] (`cargo install --path . --force --features vertex` — no `--bin` filter, must include vertex feature), [[feedback-vertex-burst-suspension]] (Sashiko fan-spawn burst trips Google abuse-detection in hours; suspends project, then account).

### 2026-06-07 13:18 — Sashiko vertex.rs + prompts.rs (Sashiko internal) — two bugs fixed

While diagnosing why the mt9m113 v8 Gemini-3.1-pro escalation kept reporting "Stage 1 failed to produce valid 'concerns' and 'dismissed_concerns' arrays after 3 attempts", root-cause was identified as TWO separate bugs:

1. `vertex.rs:112 build_endpoint_url` — region=`global` was emitting `https://global-aiplatform.googleapis.com/...` (a non-existent hostname returning a generic Google 404 HTML page). Real Vertex global endpoint is `https://aiplatform.googleapis.com/...` (no region prefix in the host). Fixed; test_endpoint_url_global_claude assertion updated.
2. `prompts.rs:1938-1979` inner retry loop — was retrying Fatal-classified upstream errors (404, 401, 400) 3× before bubbling up a generic stage-failure message that buried the real cause. Now bails immediately on Fatal `ClaudeError`, EXCEPT when error contains "RECITATION"/"blocked" (those still route to the prompt-perturbation recovery path that the existing test_recitation_error_triggers_prompt_perturbation covers).

Plus: `gemini-3.1-pro-preview` model is access-listed at `global` only on this project (us-central1 returns 404 NOT_FOUND). Settings.toml updated to `region = "global"`.

All 259 lib tests pass. Binary rebuilt.

### 2026-06-07 — mt9m113 v8 prep — 8 driver + 1 binding Sashiko gemini-3.1 findings folded

8 driver fixes amended into `submit/media-mt9m113` (held local):

| # | Finding | Loc |
|---|---|---|
| 1 | `__v4l2_ctrl_handler_setup` called without holding hdl->lock → switched to `v4l2_ctrl_handler_setup` and dropped misleading "V4L2 core holds the mutex" comments | ifp_registered |
| 2 | start_streaming retry loop didn't power-off after final failure → guard with `!chip_off`, set chip_off on power-off-failure path | start_streaming |
| 3 | ifp_s_stream had no protection against concurrent ioctl during remove → take `sensor->lock` and check `dying` flag, bail with `-ENODEV` | ifp_s_stream |
| 4 | pa_pad_ops missing set_fmt → added pa_set_fmt that returns fixed format unchanged | pa_pad_ops |
| 5 | atomic_t release_count refcount ordering wrong on error path → atomic_inc BEFORE v4l2_async_register_subdev; atomic_dec on register failure | ifp_registered |
| 6 | test_pattern_active cleared AFTER mt9m113_refresh; refresh may apply stale state → reorder to clear BEFORE refresh | ctrl-handler op |
| 7 | remove() called mt9m113_power_off unconditionally — could power-off a chip already runtime-suspended | mt9m113_remove |
| 8 | bridge_attached flag added — snapshot `was_bridge_attached` BEFORE async_unregister, then conditionally call mt9m113_release_sensor (avoids UAF after async_unregister destroys the subdev) | mt9m113_remove |

Plus binding fix: added `allOf: - $ref: /schemas/media/video-interface-devices.yaml#` to aptina,mt9m113.yaml.

Build clean. dt_binding_check clean. Held local; will not be sent until Sonnet + Gemini-3.1 escalation verdict + ideally maintainer feedback on v7 (1-2 weeks).

### 2026-06-02 15:52 — `clk-gdsc-preexisting-fixes` r1 (Vertex gemini-2.5-pro) — 4 findings, all preexisting:true ✅

4 findings, 3 patches in Set B (PATCH 3/3 fixes two distinct adjacent bugs in `gdsc_unregister` — kept as one commit per user decision):

| # | Finding (preexisting:true) | Loc | Set B fix |
|---|---|---|---|
| 1 | `gdsc_unregister` missing `pm_genpd_remove` → resource leak; pm_genpd_init re-probe would -EEXIST | gdsc.c:653 | PATCH 3/3 |
| 2 | `gdsc_unregister` tears down subdomains **before** of_genpd_del_provider → race window where a consumer attaches to a domain mid-removal (UAF risk) | gdsc.c:649 | PATCH 3/3 (same commit, reorder + add pm_genpd_remove loop) |
| 3 | `gdsc_poll_status` treats negative `gdsc_check_status` errno as boolean true → returns 0 (success) on regmap failure | gdsc.c:109 | PATCH 1/3 |
| 4 | `gdsc_init` ALWAYS_ON branch discards `gdsc_enable` return value → registers genpd as ON when silicon is OFF | gdsc.c:483 | PATCH 2/3 |

All 4 are flagged `preexisting: true` — equivalent to a clean review for a "preexisting fixes" series. review_inline produced a maintainer-style reply asking clarifying "did the original do X?" questions; useful for the cover letter narrative.

324K tokens in / 10K out / 50K cached, 18 min. Full log: `/tmp/sashiko-gdscB-vertex2.log` (purged).

### 2026-06-02 12:30-13:00 — multiple branches (claude-cli, both Opus + Sonnet) — Stage N invalid-arrays + 0 tokens

- gdsc-msm8x60-r3, gdsc-preexisting-r1, mmcc-r8, interconnect-r10, cy8c-r7 all failed with `Stage N failed to produce valid 'concerns' and 'dismissed_concerns' arrays after 3 attempts` and `0/0/0` tokens.
- Cause: 5h claude-cli rate window exhausted earlier in the morning by the Stage 1 confirmation chain. Falsely looked like a sashiko bug for several hours.
- Fix: switched provider to Vertex; auth tripped on ADC project mismatch; corrected to `project-fbffa5c0-2f85-4d78-8be`; first Vertex run succeeded.

### 2026-06-02 10:40 — `clk-mmcc-msm8660` r8 (claude-opus-4-7) — clean

- 596 KB JSON; no concerns. Validated the unhalt fabric -EPROBE_DEFER + GFX2D[01]_AHB_RESET v2 rework.

---

## Next-step backlog

1. ~~Sonnet retry verdict for mt9m113 v8~~ — CLOSED. See Findings Log 2026-06-07 17:50.
2. **Sashiko sweep for the 8 newly-tracked branches** (Stage 4: camss/rotator/vpe/vidc; Stage 5: 4 IIO). At least one of (camss-msm8660, rotator, vidc, vpe) has never been reviewed since the most recent fold. **Use `claude-cli` not Vertex** — see [[feedback-vertex-burst-suspension]]; Vertex is dead until Google account un-restricted.
3. **Stage 1 final sends** (rows 4, 5, 6) — interconnect, mpm, pm8901-tm — all Sashiko-clean, just pending send.
4. **gcc-cleanup v2 (#149)** — fold YAML pll4 + clk-pll NULL-check separate, then hold for lkml v1 feedback before send.
5. **qce-msm8660 r2 (#114)** — 3 Criticals + 4 High to address before send.
6. **mmcc v2 reroll** — drafted; blocked on gdsc-msm8x60-legacy (#0) v2 landing.
7. **A6 5-way split (#131)** — in progress.
8. **LM8502 r2 confirmation (#128)** — mfd + haptic.
9. **STAGES.md update** — Stage 4 (multimedia) is in STAGES.md but the IIO sensor patches (Stage 5 here) are NOT yet. Consider folding them into STAGES.md too, or formalising them as a separate "tenderloin sensor pre-req" track.
