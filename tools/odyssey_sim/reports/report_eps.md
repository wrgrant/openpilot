# Odyssey EPS preparation report

The offline preparation is ready for review: the image tool reproduces the stock and community images exactly, two milder research images are outside git, the fork recognizes the modified EPS and scales commands down by default, and the flashing kit now validates this firmware's checksums. **Nothing was flashed, no flasher was run, and neither the comma device nor the van was accessed. No commits, branch switches, staging or pushes occurred.**

My recommendation is to finish the lead's code review and choose one image/configuration pair before considering a flash. Do not interpret this preparation as a recommendation to install the experimental community firmware. The low-speed fade, the two changed words' semantics, and EPS override/fault thresholds remain unproven. Dividing commands by the firmware multiplier is implemented as requested, but does **not** establish that the first drive feels stock: the firmware scales a nonlinear table's outputs rather than its inputs. Physical stock-equivalence needs measurement.

## Proven and unknown

Proven offline: codec round trips; byte-exact community reconstruction; all image checksums; known payload change locations; modified identity recognition; default CAN scale-down; validated map fallback/ceiling; unmodified final CAN command; inverse-fit behavior on a known linear example and refusal without post-flash coverage; flasher source sequence; upstream source equality before our checksum entry.

Unknown: actual motor response of any candidate; whether retaining the two 1024 words constrains gains; where low-speed fade executes; driver override thresholds; EPS torque-rate/fault checks; on-car flash/dependency acceptance and recovery; steering dynamics and body motion after a flash. Hardware motor headroom under manual assist does not prove lane-keeping headroom is safe or usable.

## 1. Image tool and candidates

New repository files: `tools/odyssey_eps/{rwd.py,prepare.py,fit.py,test_rwd.py,test_fit.py,README.md}`. No Honda binary is in that directory or added to git.

`Image.decode/encode` preserve all six length-prefixed headers and the initial three bytes, enforce THR-A020 application headers, expected EPS address and cipher keys, flash address 0xC000 and payload length 0x74000, and verify both payload sums and the container checksum. The cipher is a full byte permutation and its inverse. The lookup was extracted as a literal with Python AST from the reviewed checker; downloaded modules were not imported. Malformed/truncated inputs fail with ValueError.

`build` accepts eight nonzero output knots or a scale, two words and the exact stock/modified identity. It updates all six table copies and fourteen payload identities, then recomputes the BE16 sum at 0x73f80, BE16 negative sum at 0x73ffe, and LE32 byte-sum trailer. Offsets in this report are decoded payload offsets, **not RWD file offsets**. The table includes a separate leading zero output at table-2; input axis is at table-20. The two changed words are table+22 and table+24, not directly adjacent to the table. Headers already contain BOTH stock and modified identities and equal secrets; they stay unchanged.

Acceptance passed:

| Image/check | SHA-256 |
|---|---|
| Stock round trip AND stock-to-stock build | `90cb1d23d8a27f4c45d52fcbc0a6f08813fb2ae87a0c2f9069f552edd7715edb` |
| Stock rebuilt with community outputs, words 2560/2560, comma identity | `7086b01cf9c78b4a094662d8f9f0c9fb915d1ea497d90d56f21c0224c986b9b8` |
| Candidate uniform 1.5x | `ab2a5336e3997cb5b7952a3c0186c0ebcdafaf02439bdc56f93b91078e0a1132` |
| Candidate uniform 2.0x | `5b30a4606e23f15c810f3ce949bb10bac3dd5445f5ed46cfb1ecd9fc62c560bd` |

Candidates, SHA256SUMS and README: `~/comma-logs/odyssey-eps/images/`. First nonzero knot scales uniformly with the others, unlike the community image's smaller multipliers at its first two knots. Zero stays zero, integer rounding is nearest-even. Two unknown words remain 1024/1024 because changing something with unknown semantics is unjustified; this is intentionally different from the community build acceptance and may limit effective authority. The labels describe table gain, **not measured motor gain**.

Synthetic tests need no firmware. Both reference files must exist or the external reference test skips. Both actual reference hashes were checked here.

## 2. Calibration analysis

Read stock and community payloads with our isolated codec. Saved six 768-byte neighborhoods in `~/comma-logs/odyssey-eps/calibration_inventory.json`. The same relative tables recur in the six copies. Inspected full neighborhood words and searched the first neighborhood for monotone six-word Q8-like ramps ending at 256; none matched the increasing fade hypothesis. This is a bounded search, not exhaustive reverse engineering. No disassembler or control-flow references were established.

Candidates below are offsets for the first torque table at 0x4bbc2; corresponding relative offsets recur in the other five copies.

| Candidate | Evidence and interpretation | Confidence |
|---|---|---|
| Axis at 0x4bb94: 0,64,128,192,256,320; row at 0x4bba0: six 256s | If Q8, unity everywhere. Cannot directly explain 15% to 100% fade. It could be speed-indexed but units and consumers are unknown. | High confidence this row alone is not the measured increasing fade; low confidence about its purpose. |
| Axes 0x4bb54 and 0x4bb74: 0,341,518,623,735,826,889,1281; rows at 0x4bb64/84: 256,256,166,117,62,20,0,0 | Paired decreasing Q8-shaped gains, plausible speed attenuation or assist shaping. Direction is opposite the measured low-speed increasing fade for a positively increasing speed axis. | Low candidate for speed shaping; no match to fade. |
| Axes at 0x4bbdc/0x4bbfc: 0,182,259,301,385,567,714,812; rows 0x4bbec/0x4bc0c: 256,256,256,247,214,126,51,0 | Paired decreasing gains. Plausible normal-assist shaping, not a direct increasing low-speed multiplier. | Low; no variable/units reference. |
| 0x4bc1c: 0,24,272,288,304,320 → 256,256,102,102,102,102; 0x4bc34: 0,16,160,192,256,320 → 256,256,102,77,77,77 | Other decreasing gain schedules; possible speed, torque, temperature or mode dependence. Shape does not identify the input. | Low; not a justified fade edit. |
| 0x4bc70: 0,55,166,319,532,776,887,942,998 → 1,4,6,8,10,11,11,11,11 | Increasing saturation could superficially resemble a fade after rescaling, but output ratio begins ~9%, input meaning/scaling unknown, and this could be a torque/rate schedule. | Very low; no match established. |

**No low-speed fade location was identified.** The observed 15%,30%,40–60%,100% relationship is insufficient to identify bytes without knowing the input variable, units and interpolation/control path. The unity row is not evidence that low-speed fade is absent.

The words 1024→2560 at 0x4bbd8/0x4bbda scale exactly with the community's nominal 2.5 multiplier. Nearby words are 410,32765,640; this gives no semantic proof. A paired positive/negative bound, a rate pair or fixed-point gain pair are all possible. Matching multiplier is circumstantial evidence for command-path scaling, **not proof of a limit, rate or filter**. They remain unchanged in mild candidates.

No EPS-side driver-override comparison, torque-rate check or fault threshold was identified. Table-adjacent constants cannot honestly answer that question. An exact next reverse-engineering step is SuperH code references to flash addresses 0x57bd8/0x57bda (payload offsets plus 0xC000), followed back to their input variables; confirm code/data mapping before treating those as pointers. No unknown calibration was edited.

## 3. Fork support and safety

Uncommitted opendbc changes are limited to:

- `opendbc/sunnypilot/car/honda/fingerprints_ext.py`: comma identity in Odyssey EPS FW versions.
- `opendbc/sunnypilot/car/honda/eps_mod.py`: validated `OdysseyEpsMap`, modified-Odyssey-only initialization, post-map slew.
- `opendbc/car/honda/carcontroller.py`: three-line hook after the existing mapping.
- `opendbc/sunnypilot/car/honda/test_honda.py`: map and transmitted-CAN tests.

Existing interface logic sets EPS_MODIFIED on a comma in EPS firmware, so no redundant interface edit was needed. Unmodified Odyssey takes the unchanged code path. Modified Odyssey defaults to command/2.5 (integer truncation) and a stock-command-proxy ceiling of 1.0. Actual full-scale CAN commands tested: stock -4096, modified -1638. The controller test checks a 40-frame startup ramp as well as the endpoint. Prior Civic EPS tests still pass.

JSON configuration is the direct file parameter `/data/params/d/OdysseyEpsMap`, read once at initialization. This bounded task cannot add a registered key/UI in main-repo param files; the README documents that integration limitation. Missing/bad JSON, non-finite/invalid arrays, unsupported scale, bad axes and excessive ceiling fall back to default. Valid fields: firmware_scale (1.5/2/2.5), authority_ceiling (default 1, maximum 2), speed_bp/speed_gain, torque_bp/torque_v, optional per-speed torque_rows. Limits apply after gain and interpolation. Speed-gain handles future fade compensation; nonlinear torque knots allow extra authority only at the top. See README for schema/example. This is not installed anywhere.

**Scale identity ambiguity:** all supplied images use the same comma ID. The default assumes the community 2.5 image. A 1.5 or 2.0 candidate requires explicit matching firmware_scale before driving; fingerprint alone cannot infer it. Bad config always assumes 2.5, so physical stock-equivalence cannot be promised for other images. Even community image outputs are not uniformly 2.5 at the first two knots.

`honda.h` steering TX checks only whether lateral/overall controls are allowed before permitting nonzero torque; it does not provide a Honda torque-magnitude, driver-torque or slew envelope here. No safety code was changed. Original `STEER_DELTA_UP/DOWN=3` is normalized units/s, applied before mapping at 100 Hz. For a constant divided map, the existing normalized rate automatically gives 1/scale CAN slew, so do not multiply it upward. Speed-dependent/nonlinear gain could otherwise bypass it: an additional post-map bound uses STEER_MAX × STEER_DELTA_UP × DT_CTRL / scale. This deliberately preserves a conservative CAN-command rate rather than allowing a faster rate with higher authority. It does not prove physical EPS motor-rate equivalence.

## 4. Measurement and step-up plan

`tools/odyssey_eps/fit.py` characterizes ACTUAL CAN command versus unsigned motor by speed, excluding driver input and non-steady blocks. It reports independent one-second block counts, median and 10th/90th percentile; then monotonically inverts measured curves into the JSON grid. It requires three blocks per needed cell, sufficient stock target coverage, at least four modified knots and two speed bins, and refuses plateaus, excessive monotonic corrections or targets outside the observed response. Initial output remains at command-proxy ceiling 1.0. It pools directions; review held pulses left/right separately. An emitted JSON is a review candidate, not an automatic installation.

Exercised using real stock caches and no post-flash directory: `scratchpad/eps/fit-baseline/fit_report.json` reports insufficient coverage and emits no map. A known linear synthetic plant gives the correct inverse map in tests. No post-flash measurements are claimed. A short ordinary drive may not cover enough cells; retain the default and collect more instead of extrapolating or forcing dangerous maneuvers. Interpolation across speed bins is provisional and should only be used within covered speeds.

After a separately approved flash, use a controlled setting and attentive operator with immediate takeover ability. Start with matching-scale default, ceiling 1.0; first verify assist, direction, faults and manual override while stationary/at walking speed. If it is stronger or weaker than stock, stop increasing authority and characterize the response. No hands-off road test is justified by offline evidence.

1. Record normal gentle driving through available speed bins at default settings. Use the unchanged Odyssey suite only in its intended 38–80 mph operating range, on an appropriate road/test setting, to measure onset/delay and symmetry. Its small pulses cannot establish full-scale motor gain or low-speed behavior. Do not edit its speed gates to chase missing bins.
2. Extract CAN caches to a new directory under `~/comma-logs/odyssey-eps/` using the existing `tools.odyssey_sim.plant.extract.extract(route)` helper with its SCRATCH redirected to that directory and a `plant_data/` child created there. Its legacy liveParameters branch does not populate the newer vehicleParameters extras; fields needed here (cmd,motor,v,active,pressed,drv,t) come from CAN/carControl/carState. Never run its unredirected default writer for this task.
3. Run `.venv/bin/python -m tools.odyssey_eps.fit --stock ~/comma-logs/odyssey-evidence/plant_data --modified POSTFLASH_CACHE_DIR --scale SELECTED_SCALE --out NEW_RESULT_DIR` from the repo. Run `tools/odyssey_eps_report.py ROUTE_DIR` for held-pulse timing and left/right response. Reject driver-contaminated data and use separate drive data to validate the inverse. Motor is unsigned; use wheel rate/angle and actual body response to confirm direction, not motor sign.
4. Only after measured stock equivalence and manual override are satisfactory, separately approve ceiling 1.10, then 1.25, then 1.50; 2.0 is a later decision, not an automatic target. Change one parameter at a time offroad, keep gain=1 in the normal command range, add authority only at the top, and remeasure before each increase. Treat low-speed compensation as a separate experiment and keep within measured/approved physical torque, not merely a larger command ceiling.
5. At each step compare command→motor, wheel rate, onset/release delay, body lateral motion/jerk, tracking, saturation, driver intervention and EPS status against stock-equivalent baseline. Use steering-derived lateral acceleration or validated vEgo×yaw-rate, never VEHICLE_DYNAMICS.LAT_ACCEL. Stop for any EPS fault/temporary unavailability, unexpectedly large or delayed torque, inability to override promptly, motor mismatch, growing body oscillation/jerk, lane departure, or unreliable logs. Restore the last measured map; if assist/fault state is wrong, stop driving and diagnose rather than “tuning through” it.

These steps plan future work only. No map or suite was deployed. A fitted torque curve cannot prove the closed-loop system handles tighter curves safely.

## 5. Flashing kit review

Read `eps-update.py`, `flash.py` and `eps-diag.py` completely in bounded source reads, including remainder of the guided menu. Nothing invoked their vehicle entry points. Kit is `~/comma-logs/odyssey-eps/kit/` with source, parser package, matching stock image, source snapshots/diffs, README and SHA256SUMS. Candidates remain outside the guided menu. Kit README contains pre-flight, sequence, recovery and failure modes.

Source-level sequence: Panda elm327 mode, bus 1 default (actual bus unverified), request address 0x18DA30F1 / reply 0x18DAF130; tester-present, F181 ID read; extended session 03; seed/key security; programming session 02. THR secrets in both header entries are `01 11 01 12 11 20`, so k0=0x0111,k1=0x0112,k2=0x1120 and key=((seed+k0) XOR ((seed*k1) mod k2)) & 0xffff, BE16; seed uses final two bytes. App-ID comparison normalizes '-' and ',' and removes trailing NULs.

Only with --danger: routine-start FF00 erase; DID F101 cipher keys 01 02 03; download 0xC000/0x74000; encrypted transfer blocks with incrementing wrapping byte sequence and negotiated size minus two bytes; transfer-exit; routine-start FF01 dependency check; hard reset. Live dry run reaches PROGRAMMING before aborting, then attempts default session. It can change diagnostic/security state and does not validate write/dependency/recovery. It was not run.

Added THR checksum definitions to both external `dl/tools/eps-update.py` and `check_rwd.py`, then copied those reviewed sources into kit. Only the read-only checker was run with `-I`; both mild images pass all three sums. `eps-update.py`'s x5a parser itself does not check container checksum, so offline checking first remains necessary; `--skip-checksum` and Python -O defeat intended validation. Unknown firmware lengths can still skip validation in the generic source; our THR entry prevents that for this image.

Flasher failure findings: broad Panda exception → mock fallback (real flash explicitly rejects mock); Pilot-specific stale mock assertions mean it is not a THR simulator; security delay retries only NRC36/37, not invalid key35; wrong identity/secret/bus, session failure, erase/download/transfer/dependency/reset failure return nonzero; default-session cleanup is best effort. Guided helper can select an unresponsive bus, offer invalid-image checksum bypass, and retry real flashing without a second FLASH confirmation. No such escape hatch is recommended. Diagnostic tool's bootloader label is a hypothesis; silence at an ISO-TP reply address proves little about normal application health. It can leave an extended session active.

Recovery path is the checked exact stock image, after diagnosing original failure and confirming stable power/CAN, using the same approved session procedure, then verifying stock identity and assist. Both IDs/secrets in stock header support lookup from modified ID. **Recovery has not been tested and is not guaranteed.** If diagnostic/programming access is gone, stop; obtain an EPS-specific bench recovery procedure or replacement instead of blind retry.

Source comparison: GitHub tree API resolved nrdr-development to `622a1479bd27598a949d5ca601e486380b0369a0`. All three local files were byte-identical to [upstream tools at that pinned commit](https://github.com/nrdr/openpilot/tree/622a1479bd27598a949d5ca601e486380b0369a0/openpilot/nrdr/tools/eps) BEFORE our checksum entry. Saved snapshots/diffs document that baseline. Only eps-update checksum entry differs afterward; flash/diag unchanged. Community firmware provenance supplied by the brief: [jasondefuria/openpilot_ody](https://github.com/jasondefuria/openpilot_ody). No external firmware was downloaded in this run. GitHub web fetch failed; the read-only GitHub API/raw-content fallback succeeded.

## Verification, scope and handoff

Final required checks: `.venv/bin/python tools/test_runner.py tools/odyssey_eps opendbc_repo/opendbc/sunnypilot/car/honda/test_honda.py` → **16 passed**. Ruff over all five Python tool/test files and all four edited opendbc files → **all checks passed**. `git diff --check` in main and submodule → clean. Read-only kit checker validates both candidates. Existing main-repo `msgq_repo` untracked-content marker was present before work and untouched. Repo remains odyssey-sim; opendbc edits uncommitted. No new generic ARCHITECTURE.md was written because the brief restricts write paths; the existing ODYSSEY code map and scoped README supply the addition's map. No unrelated onboarding or vault changes were made.

A test initially failed because of an incorrect assumption about header IDs, then passed after reading the actual header. Another synthetic test failed because a 13-byte test slot was used for a 14-byte ID; correcting its length fixed it. Tests also first failed for absent new codec/map implementations, then passed. These failures and fixes did not involve hardware.

**Exact next step for the lead:** review the four opendbc diffs and the scoped tool files; choose ONE image and matching map scale, explicitly acknowledge that command scale-down is not proven stock motor response, and resolve whether retaining the unknown words makes the milder candidate the preferred experiment. My recommendation is the uniform 1.5x candidate with its words unchanged as the smallest firmware experiment, with explicit scale 1.5 configuration and no authority increase; it may fail to add useful torque, which is data. Before approving any vehicle action, review the kit pre-flight/recovery limitations and plan the controlled measurement session. After approval, an operator starts with battery charger, identity/bus confirmation and the live dry run; no automated agent should silently proceed from this report to programming.
