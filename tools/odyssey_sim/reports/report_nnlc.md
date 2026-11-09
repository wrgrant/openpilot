# Odyssey NNLC open-loop replay research

NNLC uses future plans, but these replays do not establish that it steers this van through curves better. Its commanded torque timing, saturation and reversals can be compared; the actual vehicle motion belongs to the recorded NNLC drives. A stock counterfactual tracking result requires worker 3’s closed-loop simulation or an on-road A/B.

Across all three drives, **NNLC shows no detectable command lead over stock at 20+ mph** in the ramp timing fit (0.05 s resolution). It reverses torque more often in every speed band. Below 10 mph it saturates **83.0% versus 38.8%** of engaged hands-off time; at 10–20 mph, **48.9% versus 28.6%**. At 20–35 mph it reverses **2.50 versus 0.73 times/s**. These results support William’s impression of no clear anticipation benefit and potentially rougher low-speed control. They do not prove worse closed-loop tracking.

## Replay validation

All three primary routes have NNLC enabled and EnforceTorqueControl=0. ControlsExt.initialize_lateral_control therefore selects the real sunnypilot **v0** LatControlTorque, not the upstream-derived v1 class. Both classes and the real extension/JSON are supported by the harness. The main comparison uses v0 to hold software version fixed.

| Route | Engaged, no driver torque frames | Correlation | RMS torque error | 99th percentile absolute error |
|---|---:|---:|---:|---:|
| nnlc_1aa | 112,417 | 0.9993502 | 0.01267 | 0.00340 |
| nnlc_1ab | 122,267 | 0.9986634 | 0.01808 | 0.00355 |
| nnlc_1a7 | 75,460 | 0.9989353 | 0.01877 | 0.00555 |

Torque is normalized ±1. Validation includes all engaged samples without steeringPressed, including transitions. Near-perfect typical agreement is not bit-exact reproduction: asynchronous subscriber timing, latest calibration/model delivery, floating point, and state at gaps leave residuals. The replay never seeds its integrator from logged PID terms.

Inputs follow the existing process-replay framework’s selfdriveState poll boundary (MessageBasedRcvCallback in process_replay.py). Messages are sorted within each segment, inputs snapshotted on selfdriveState, then paired controlsState/carControl supply the logged target and validation output. Full controlsd process replay was inspected but not run: the reusable harness isolates the torque controller and avoids rerunning longitudinal/planning processes. There is no livePose dependency: the minimal calibrated_pose argument reconstructs only orientation.pitch from carControl.orientationNED[1], which controlsd publishes from its calibrated pose. This avoids inventing pose or setting roll to zero. Calibrated pitch comes from carControl.orientationNED; VM stiffness/ratio/offset/roll and filtered lateralTorqueParameters are refreshed from the stream. Inactive and driver-torque frames still advance controller state. Analysis uses carControl.latActive AND NOT steeringPressed.

## NNLC versus stock: logged delay, pooled routes

“Tracking RMS” below is the recorded desired-minus-actual lateral acceleration (m/s²), identical evidence for both replays. It is not a stock tracking prediction. Saturation is |command|≥0.999, rather than the controller’s speed/timer-dependent saturated flag. Reversals use ±0.01 torque hysteresis and do not bridge disengagements or speed-band changes.

| mph | Engaged seconds | Recorded tracking RMS | NNLC saturation % | Stock saturation % | NNLC reversals/s | Stock reversals/s |
|---|---:|---:|---:|---:|---:|---:|
| 0-10 | 116.7 | 0.073 | 83.04 | 38.84 | 2.116 | 1.388 |
| 10-20 | 215.1 | 0.183 | 48.94 | 28.55 | 2.915 | 0.804 |
| 20-35 | 578.0 | 0.116 | 9.38 | 7.16 | 2.503 | 0.732 |
| 35-50 | 1168.9 | 0.071 | 0.78 | 1.05 | 1.654 | 0.917 |
| 50+ | 1022.6 | 0.074 | 0.00 | 0.00 | 1.263 | 0.938 |

## Anticipation at curve entries and exits

Timing is a **descriptive best-fit command lag relative to the planned acceleration ramp**, not an EPS response delay or road-curvature preview. Negative means torque changes earlier; positive means later. After 0.31 s box smoothing, entry samples have |planned accel|>0.1 m/s² and increasing magnitude >0.05 m/s³; exits have decreasing magnitude <−0.05. Fit the maximum signed derivative correlation between −command and planned acceleration over ±1 s in 0.05 s steps, separately by speed. Exclude 1.1 s around overrides/disengagements and discontinuous timestamps. These are ramp samples, not independent curve counts. Feedback, friction and saturation affect the fitted lag; weak correlation or a search-edge optimum is inconclusive. Below 10 mph there are zero eligible entry samples and only 16 eligible exit samples (0.16 s), so no timing estimate is reported. At 10–20 mph the NNLC entry correlation is weak and the −0.95 s exit fit is near the search edge, so neither supports a reliable preview claim.

| mph | NNLC entry lag s (r) | Stock entry lag s (r) | NNLC exit lag s (r) | Stock exit lag s (r) |
|---|---:|---:|---:|---:|
| 0-10 | insufficient samples | insufficient samples | insufficient samples | insufficient samples |
| 10-20 | +0.00 (0.19) | +0.00 (0.46) | -0.95 (0.41) | +0.00 (0.75) |
| 20-35 | +0.00 (0.27) | +0.00 (0.72) | +0.00 (0.22) | +0.00 (0.72) |
| 35-50 | +0.00 (0.47) | +0.00 (0.75) | +0.00 (0.48) | +0.00 (0.75) |
| 50+ | +0.00 (0.70) | +0.00 (0.82) | +0.00 (0.81) | +0.00 (0.87) |

## Changing the delay: 0.32 versus 0.45 seconds

Both core lat_delay and the extension’s update_lateral_lag are changed together. Plans, speed, steering, safety-limit history and measured motion stay fixed. In v0 the setpoint algebra collapses to current desired acceleration: stock delay has effectively no effect on output; NNLC delay changes its future sample locations and jerk features. This differs materially from v1’s delayed error setpoint.

| mph | NNLC sat % .32 / .45 | Stock sat % .32 / .45 | NNLC flips/s .32 / .45 | Stock flips/s .32 / .45 | NNLC entry lag .32 / .45 | NNLC exit lag .32 / .45 |
|---|---:|---:|---:|---:|---:|---:|
| 0-10 | 83.01 / 83.40 | 38.84 / 38.84 | 2.133 / 2.133 | 1.388 / 1.388 | insufficient | insufficient |
| 10-20 | 48.87 / 49.32 | 28.55 / 28.55 | 2.892 / 3.003 | 0.804 / 0.804 | +0.00 / +0.00 | -0.95 / -0.95 |
| 20-35 | 9.32 / 9.68 | 7.16 / 7.16 | 2.472 / 2.861 | 0.732 / 0.732 | +0.00 / +0.00 | +0.00 / +0.00 |
| 35-50 | 0.77 / 0.83 | 1.05 / 1.05 | 1.613 / 2.204 | 0.917 / 0.917 | +0.00 / +0.00 | +0.00 / +0.00 |
| 50+ | 0.00 / 0.00 | 0.00 / 0.00 | 1.255 / 1.624 | 0.938 / 0.938 | +0.00 / +0.00 | +0.00 / +0.00 |

At 35–50 mph, 0.45 s raises NNLC reversals from 1.61 to 2.20/s (+37%); at 50+ mph, from 1.25 to 1.62/s (+29%). These open-loop results give no reason to prefer 0.45 s for NNLC on anticipation or smoothness. Actual closed-loop delay compensation can behave differently.

## Upstream-derived v1 sensitivity (1aa only)

To exercise the exact class named in the brief, the real openpilot/selfdrive/controls/lib/latcontrol_torque.py is also replayed on 1aa at both delays. This class has lower highway KP/KI, a delayed feedback setpoint, and jerk-assisted friction. It is a different controller from the logged v0; it cannot validate against these logs as if version were unchanged. Low-speed angle, gain cap, deadband and all experimental toggles remain off.

| mph | NNLC sat % .32 / .45 | Stock v1 sat % .32 / .45 | NNLC raw flips/s .32 / .45 | Stock v1 raw flips/s .32 / .45 |
|---|---:|---:|---:|---:|
| 0-10 | 90.00 / 89.96 | 37.96 / 35.67 | 1.72 / 1.66 | 1.22 / 1.41 |
| 10-20 | 57.62 / 57.62 | 46.73 / 45.65 | 2.42 / 2.77 | 0.92 / 0.84 |
| 20-35 | 3.00 / 3.29 | 0.95 / 0.69 | 3.25 / 3.65 | 1.58 / 1.64 |
| 35-50 | 0.91 / 0.93 | 1.31 / 1.59 | 2.36 / 3.03 | 1.85 / 1.64 |
| 50+ | 0.00 / 0.00 | 0.00 / 0.00 | 2.46 / 2.55 | 2.52 / 2.19 |

Raw flips in this supplementary table have no hysteresis, so their absolute rates are not directly comparable to the main table. The v1 results reinforce that changing version and changing NNLC are distinct experiments.

## What the network actually changes

The Odyssey JSON maps speed, desired lateral acceleration, error/jerk/friction features, roll, past desired acceleration/roll and four future planned accelerations/rolls to torque. The future offsets are 0.3, 0.6, 1.0 and 1.5 s plus extension lag (and an acceleration adjustment). At 0.32 s lag that reaches about 1.82 s ahead. It is a learned torque/feedforward model, not a replacement path planner, camera model, or vehicle trajectory optimizer. It cannot make the upstream planned curve release sooner.

NNLC also replaces the feedback error with the difference of two network evaluations and changes PID limits into torque units. It retains the large low-speed KP schedule; it is not simply “stock plus preview.” The base PID update still executes before the extension updates that same PID again. NNLC also blends stronger error response above 1–2 m/s² and adds a friction override when the model requests it. These combined changes prevent attributing every output difference solely to lookahead.

## Calibration and A/B checks

| Route | latAccelFactor p05 / median / p95 | friction p05 / median / p95 | offset p05 / median / p95 |
|---|---:|---:|---:|
| nnlc_1aa | 1.5527 / 1.5658 / 1.5833 | 0.1354 / 0.1367 / 0.1415 | -0.0343 / -0.0330 / -0.0316 |
| nnlc_1ab | 1.4726 / 1.5510 / 1.5753 | 0.1451 / 0.1455 / 0.1477 | -0.0623 / -0.0585 / -0.0444 |
| nnlc_1a7 | 1.6014 / 1.6124 / 1.6164 | 0.1372 / 0.1384 / 0.1389 | -0.0416 / -0.0388 / -0.0339 |

The quoted factor≈1.47/friction≈0.145/offset≈−0.055 is not universal across these logs; use the actual learned values above. The older highway routes 18c/19b/19c also have NeuralNetworkLateralControl=1 in initData, but were not pooled because software revisions differ. Manual-only 1a4/1a5 are excluded.

All primary routes have SteerDeadbandCompAB=1. Crucially, git-show of the exact logged commits and the real v0 source confirms there is no SteerDeadbandComp call in v0. The selected controller therefore does not execute kick/bump at all; those settings only affect v1. The main result is not a kick/bump A/B mixture. For caution, the off-block-only sensitivity table below excludes monotonic 60-second blocks 1 and 2 of each three-block cycle.

| mph | Off-block seconds | NNLC sat % | Stock sat % |
|---|---:|---:|---:|
| 0-10 | 37.2 | 83.65 | 37.39 |
| 10-20 | 50.1 | 20.52 | 17.64 |
| 20-35 | 189.6 | 12.20 | 8.73 |
| 35-50 | 450.7 | 1.62 | 1.97 |
| 50+ | 388.7 | 0.00 | 0.00 |

## API and reproducibility

Created `tools/odyssey_sim/replay/` (untracked, no commits). Import `ControllerConfig`, `replay_route`, `replay_configs`. Example:

```python
from tools.odyssey_sim.replay import ControllerConfig, replay_route
for f in replay_route("000001aa", ControllerConfig(nnlc=True, delay=.45, version=0)):
    if f["active"] and not f["pressed"]:
        command = f["command"]  # same sign as logged torqueState.output / actuator torque
```

`delay=None` follows logged delay; version 0 matches these drives, version 1 invokes openpilot/selfdrive/controls/lib/latcontrol_torque.py. `replay_configs` evaluates multiple configurations on the same inputs, yielding command under config.name and its PID terms as suffixed keys. Rows include monotonic `t`, m/s `speed`, target/actual acceleration, activity/driver flags, logged output, calibration and delay. Route accepts a unique local substring, directory or existing file; every LogReader call receives a real FILE path. `max_frames` supports smoke tests. README documents state, timing and limitations for worker 3.

`OPENPILOT_PREFIX=simreplay` is set before Params creation. Toggle writes block until persisted and an assertion checks every controller’s loaded NNLC mode. Initialize separate replay calls sequentially to avoid racing the isolated shared Params namespace. All device/default parameters remain untouched; no SSH/device access, network, branch switching, tracked edits or commits.

Reproduce:

```sh
PYTHONPATH=$PWD .venv/bin/python -m tools.odyssey_sim.replay.run_analysis 000001aa --output /authorized/scratch/nnlc_1aa.npz
# Repeat for 1ab and 1a7, then:
PYTHONPATH=$PWD .venv/bin/python -m tools.odyssey_sim.replay.summarize /authorized/scratch/nnlc_1aa.npz /authorized/scratch/nnlc_1ab.npz /authorized/scratch/nnlc_1a7.npz --output /authorized/scratch/nnlc_metrics.json
```

Validation: one integration test via tools/test_runner.py passed; it checks logged correlation >0.999, RMS <0.002 on an early engaged sample, and confirms stock/NNLC commands differ >0.1. Ruff passes for all new replay Python files. Full-route validation above is the broader check.

## Limits and next decision

Open-loop replay cannot verify counterfactual tracking error, lane offset, driver takeover rate, actual lateral-acceleration release time, EPS torque delivery, or the benefit of changing delay in a feedback loop. Logged safety limiting is held fixed rather than recomputed for each counterfactual. It also uses recorded desired curvature, so changing delay cannot replan the curve. Sign reversals are a command roughness proxy, not physical jerk. Low-speed saturation and weak entry/exit correlations limit timing interpretation.

Recommendation: treat NNLC as an alternative learned torque mapping with preview, not a proven cure for late turn entry or slow release. Keep the matched v0/v1 distinction explicit in worker 3’s closed-loop comparison; compare both against the same vehicle/EPS delay model. Do not select a road tune solely from these open-loop timing fits.

## Residual and roughness cross-check

| Route | Frames with replay error >0.02 torque | Maximum absolute error | Logged raw flips/s | Replay raw flips/s |
|---|---:|---:|---:|---:|
| 1aa | 221 (0.197%) | 1.807 | 2.495 | 2.517 |
| 1ab | 157 (0.128%) | 2.000 | 2.562 | 2.612 |
| 1a7 | 240 (0.318%) | 1.297 | 2.202 | 2.254 |

Rare large residuals dominate RMS despite low 99th-percentile errors. Their precise cause is not fully isolated; subscriber delivery and output pairing are reconstruction assumptions, not proven explanations. Similar logged/replayed sign-change rates provide a separate roughness check. For safety, no claim of bit-exact or production-grade process replay is made.

Artifacts: `nnlc_1aa.npz`, `nnlc_1ab.npz`, `nnlc_1a7.npz` (six configurations each), corresponding route metadata JSONs, `nnlc_v1_1aa.npz`, `nnlc_metrics.json`, `nnlc_suburban_metrics.json`, and `nnlc_comparison.html` in this scratch directory. The reusable source files are `__init__.py`, `harness.py`, `run_analysis.py`, `summarize.py`, `test_replay.py`, and `README.md` under tools/odyssey_sim/replay/.
