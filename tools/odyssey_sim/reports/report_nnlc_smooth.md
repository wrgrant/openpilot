# Odyssey learned-steering research receipt

**Recommendation: advance S3 as the conservative review candidate, and retain S5 as the stronger straight-tracking research arm. V1 and V2 must no longer be rejected merely for torque reversals.** All candidates remain default-off; no road/device changes, commits, branch switches or pushes occurred.

## What this establishes

The owner-trained inverse model has useful closed-loop tracking performance. A tiny causal feedforward filter preserves V1 tracking while reducing command activity. Stronger torque-space feedback improves straight and curve tracking further, with a measurable output-motion tradeoff. This is evidence for an offline candidate, not a certified improvement in lane wander or driver feel.

## Research used

- [comma Controls Challenge / RL controls](https://blog.comma.ai/rlcontrols/): evolutionary parameter search directly optimizes closed-loop performance; their feature-rich feedback controller used six tuned parameters. I used deterministic grid plus seeded random black-box search over controller shaping, rather than retraining inverse labels as though their MAE were the driving objective.
- [jonoomph neural controller](https://github.com/jonoomph/controls_challenge): documents temporal state and torque-history inputs; relevant to preserving the actual causal runtime contract. Its performance claims are its author’s, not a demonstrated Odyssey transfer.
- [omarrayyann RL controller](https://github.com/omarrayyann/controls_challenge): a learned closed-loop-policy lead; no weights or claimed results transferred here. PPO/model-based MPC are alternatives, but the available dynamics and perception models remain uncertain enough that compact, bounded controller tuning is the more reviewable first rep.
- [SAE 2015-01-2729](https://saemobilus.sae.org/papers/active-return-center-control-strategy-steering-wheel-sensor-electric-power-steering-system-2015-01-2729): return-state identification from wheel velocity/torque and double-loop return compensation. [SAE 2008-01-0499](https://saemobilus.sae.org/papers/sliding-mode-return-control-electric-power-steering-2008-01-0499): friction-robust return control and assist/return switching. These abstracts motivate state-conditioned unwind; neither validates adding an unconditional centering torque while lane keeping.
- [US6408234B1](https://patents.google.com/patent/US6408234B1/en) is specifically EPS hysteresis compensation, not an empirical Odyssey return-to-center result. Its distinction between direction/history-dependent response and instantaneous torque supports treating raw command chatter separately from wheel motion.
- Local sources: ODYSSEY.md; report_van_nnlc.md; report_calib.md; BRIEF_COMMON.md; report_perception.md and perception/metric.py. Human-threshold literature is assessed by the perception report. No published sunnypilot NNLC smoothing method was established by this work; absence of a method is not proven.

## Data, objective and evaluation contract

No new supervised weights were trained. V1’s original public pretraining and owner adaptation are reused. Adaptation already included 19c, so 19c is not a held-out controller result here despite being a plant holdout. All tuning uses a7 only. Three arms were frozen in selected.json before aa/ab evaluation. The later V2 run is an untuned reference with its existing weights/history adapter, not a model selected on aa/ab.

Primary evaluator: calibrated schema-v2 William plant, logged-plan mode, actual V0/NNLC, 100 Hz updates, 10 Hz logged plans, per-frame tuning/delay/pitch, 0.1-degree feedback quantization and Honda 3 units/s applied-command slew. Public sensitivity: original public schema-v1 steering dynamics with the same calibrated controller front end and slew; this is deliberately a different, uncalibrated public plant, not a second validated v2 fit. Body-stage sensitivity remains approximate.

Selection: two a7 windows, 60 s each. Final test: two aa windows of 60 s and six ab windows of 30 s, selected by the calibration harness’s uninterrupted active/no-driver and lane-confidence rules. First 10 s warmup excluded. Eligibility excludes <=5 m/s, blinker/driver activity and gaps. Results are this selected sample, not route-wide prevalence. Tables average eligible-window metrics equally; raw n and traces are retained for pooled analyses. Cost = 100×mean squared lateral-acceleration error, not the full Controls Challenge 50×cost+jerk objective.

Straight following has priority: inspect |planned accel|<0.1 separately, then total tracking, output roughness and the alternate plant. The scalar 3×straight-cost + total-cost was inspected as a ranking aid, not used as a stand-alone safety gate. Families cover reduced gain/friction, causal error filtering, friction filtering, FF filtering, jerk-feature suppression and command slew. Random search seed 20261109. We retained Pareto alternatives instead of optimizing away William’s priority with a reversal penalty.

## Held-out aa/ab: motion and commands

| Arm | Plant | Tracking cost | Straight cost | Mid-curve cost | Raw flips/s | Output roughness |
|---|---|---:|---:|---:|---:|---:|
| Stock V0+NNLC | william | 0.3055 | 0.2182 | 0.3248 | 1.220 | 0.0000 |
| Stock V0+NNLC | public | 0.5071 | 0.3419 | 0.4937 | 0.926 | 0.0000 |
| V1 | william | 0.1004 | 0.0765 | 0.1242 | 2.457 | 0.1264 |
| V1 | public | 0.2040 | 0.1621 | 0.2056 | 2.162 | 0.0325 |
| S3 (20 ms FF) | william | 0.1004 | 0.0784 | 0.1144 | 1.552 | 0.1290 |
| S3 (20 ms FF) | public | 0.2048 | 0.1642 | 0.2074 | 1.374 | 0.0332 |
| S4 (gain 1.4, 50 ms error) | william | 0.0843 | 0.0677 | 0.0945 | 1.782 | 0.2627 |
| S4 (gain 1.4, 50 ms error) | public | 0.1517 | 0.1281 | 0.1464 | 1.520 | 0.1835 |
| S5 (gain 2, 50 ms error) | william | 0.0751 | 0.0621 | 0.0773 | 2.096 | 0.3344 |
| S5 (gain 2, 50 ms error) | public | 0.1287 | 0.1253 | 0.1180 | 1.723 | 0.3846 |
| V2 (offline causal adapter) | william | 0.1043 | 0.0809 | 0.1199 | 2.752 | 0.0624 |
| V2 (offline causal adapter) | public | 0.1843 | 0.1530 | 0.2113 | 2.064 | 0.0403 |

Roughness is the perception worker’s output-based index, not probability or an absolute comfort rating. Wheel/body channels only; hand torque is unavailable from either plant. Command flips never enter this score. Stock can score zero while tracking poorly: low activity is not sufficient. S3’s few remaining reversals do not by themselves establish discomfort, nor does a zero score prove no hand reaction.

### Threshold sensitivity

| Arm | Plant | thresholds ×0.25 | ×0.5 | ×1 | ×2 |
|---|---|---:|---:|---:|---:|
| Stock V0+NNLC | william | 0.9175 | 0.1135 | 0.0000 | 0.0000 |
| Stock V0+NNLC | public | 0.6986 | 0.0668 | 0.0000 | 0.0000 |
| V1 | william | 2.4605 | 0.8295 | 0.1264 | 0.0000 |
| V1 | public | 2.1443 | 0.7037 | 0.0325 | 0.0000 |
| S3 (20 ms FF) | william | 2.5048 | 0.8439 | 0.1290 | 0.0000 |
| S3 (20 ms FF) | public | 2.2454 | 0.7453 | 0.0332 | 0.0000 |
| S4 (gain 1.4, 50 ms error) | william | 3.1349 | 1.1545 | 0.2627 | 0.0000 |
| S4 (gain 1.4, 50 ms error) | public | 3.0368 | 1.0869 | 0.1835 | 0.0000 |
| S5 (gain 2, 50 ms error) | william | 3.7606 | 1.4966 | 0.3344 | 0.0000 |
| S5 (gain 2, 50 ms error) | public | 4.0246 | 1.6107 | 0.3846 | 0.0000 |
| V2 (offline causal adapter) | william | 2.1745 | 0.6994 | 0.0624 | 0.0000 |
| V2 (offline causal adapter) | public | 2.1613 | 0.5940 | 0.0403 | 0.0000 |

All perceptual thresholds except wheel radius are scaled together. The final report uses perception v2 (0.2 m/s² provisional rim allowance; primary bands >=0.5 Hz), superseding exploratory scores calculated while that worker was still editing v1. No hidden assumption that high EPS friction eliminates hand torque: the sensor retains command-correlated ripple, and these models do not predict it in calibrated N·m.

### Four priorities

1. **Straight/no wander:** S3/V1 substantially reduce small-plan acceleration error versus stock, with S5 the stronger straight candidate. This does not prove the 7–20 s lane loop improves: logged plans stay fixed and the calibrated lane geometry does not close. Perception’s primary bands also exclude that slow wander. Hand this result to the wander/calib3 worker as a better inner-loop candidate.
2. **Low speed:** no plant validates parking/intersection operation. This evaluator excludes <=5 m/s and lane-confidence filtering further reduces neighbourhood coverage. Runtime shaping bypasses below 5 m/s; it does not fix V0’s low-speed gain or saturation. The base V1 inference is still an experimental model there. Keep the low-speed worker’s intervention separate until the lead integrates and verifies it.
3. **Pre-entry:** tracking improves for changing plans and mid-curve, but the existing target does not command earlier than the logged planner. A 20 ms FF filter introduces a small causal lag, not anticipation. Future-plan inputs remain causal runtime plan signals. No claim of turning before a curve or reducing real takeovers follows from these scores.
4. **Exit:** no unconditional wheel centering was added. Phase-specific accel error and lag are diagnostics below; an upstream ~3 s planned decay cannot be fixed by faithfully tracking that same decay. A separate plan-conditioned unwind module needs curve-worker evidence.

### Planned-entry / exit diagnostics

| Arm | Plant | rising-demand cost | falling-demand cost | mean best-fit lag (s) |
|---|---|---:|---:|---:|
| Stock V0+NNLC | william | 0.3990 | 0.3241 | 0.469 |
| Stock V0+NNLC | public | 0.6190 | 0.5431 | 0.631 |
| V1 | william | 0.1340 | 0.1193 | 0.164 |
| V1 | public | 0.2980 | 0.2170 | 0.250 |
| S3 (20 ms FF) | william | 0.1317 | 0.1216 | 0.167 |
| S3 (20 ms FF) | public | 0.2915 | 0.2228 | 0.290 |
| S4 (gain 1.4, 50 ms error) | william | 0.1142 | 0.1028 | 0.147 |
| S4 (gain 1.4, 50 ms error) | public | 0.2179 | 0.1693 | 0.199 |
| S5 (gain 2, 50 ms error) | william | 0.1018 | 0.0949 | 0.129 |
| S5 (gain 2, 50 ms error) | public | 0.1760 | 0.1520 | 0.170 |
| V2 (offline causal adapter) | william | 0.1425 | 0.1293 | 0.177 |
| V2 (offline causal adapter) | public | 0.2571 | 0.2097 | 0.286 |

Rising/falling means derivative of |planned accel| above/below ±0.1 m/s³, not independently labelled curve events. Lag is the metric’s global reference-conditioned best shift, which can be unreliable with weak excitation. These diagnostics do not score road pre-entry timing.

## Logged-motion replay (actual runtime shaping)

First 20,000 control frames per held-out route, the validated replay adapter, real V0. Stock correlation and MAE verify the inherited replay path. Candidate command MAE means difference from the historical controller, not better tracking. Measured motion is fixed across replay arms; attributing different measured roughness to them would be invalid.

| Route / mph | Arm | n | Command MAE | Correlation with logged | Flips/s | command-rate RMS |
|---|---|---:|---:|---:|---:|---:|
| 000001aa / 20-35 | stock | 115 | 0.00000 | 1.0000 | 5.263 | 1.671 |
| 000001aa / 20-35 | V1 | 115 | 0.05538 | 0.9053 | 3.509 | 2.544 |
| 000001aa / 20-35 | S3 | 115 | 0.05395 | 0.9006 | 1.754 | 1.929 |
| 000001aa / 20-35 | S4 | 115 | 0.09673 | 0.7780 | 2.631 | 1.611 |
| 000001aa / 20-35 | S5 | 115 | 0.15704 | 0.7859 | 3.509 | 1.862 |
| 000001aa / 20-35 | V2 | 115 | 0.07557 | 0.9741 | 2.631 | 2.677 |
| 000001ab / 20-35 | stock | 28 | 0.00000 | 1.0000 | 0.000 | 2.285 |
| 000001ab / 20-35 | V1 | 28 | 0.13623 | 0.9722 | 0.000 | 4.125 |
| 000001ab / 20-35 | S3 | 28 | 0.14004 | 0.9713 | 0.000 | 3.086 |
| 000001ab / 20-35 | S4 | 28 | 0.21222 | 0.9375 | 0.000 | 2.285 |
| 000001ab / 20-35 | S5 | 28 | 0.32249 | 0.9165 | 0.000 | 2.618 |
| 000001ab / 20-35 | V2 | 28 | 0.14049 | 0.9987 | 0.000 | 4.256 |
| 000001ab / 35-50 | stock | 4590 | 0.00001 | 1.0000 | 2.594 | 4.452 |
| 000001ab / 35-50 | V1 | 4590 | 0.13773 | 0.8758 | 2.746 | 4.327 |
| 000001ab / 35-50 | S3 | 4590 | 0.13591 | 0.8774 | 1.918 | 3.298 |
| 000001ab / 35-50 | S4 | 4590 | 0.18726 | 0.8430 | 2.092 | 2.694 |
| 000001ab / 35-50 | S5 | 4590 | 0.26531 | 0.8156 | 1.809 | 3.030 |
| 000001ab / 35-50 | V2 | 4590 | 0.18515 | 0.8620 | 1.984 | 4.374 |
| 000001ab / 50-200 | stock | 6697 | 0.00012 | 1.0000 | 1.912 | 3.395 |
| 000001ab / 50-200 | V1 | 6697 | 0.11819 | 0.9235 | 2.808 | 3.944 |
| 000001ab / 50-200 | S3 | 6697 | 0.11577 | 0.9268 | 1.658 | 2.833 |
| 000001ab / 50-200 | S4 | 6697 | 0.16060 | 0.9117 | 1.792 | 2.727 |
| 000001ab / 50-200 | S5 | 6697 | 0.22757 | 0.8972 | 1.598 | 2.980 |
| 000001ab / 50-200 | V2 | 6697 | 0.16022 | 0.9654 | 1.867 | 3.594 |

## Generation log: every attempted configuration

Cells are mean tracking cost / raw flips per second / output roughness. These are a7 selection results, not held-out scores. Parameters omitted from the compact label retain Trial defaults. Failures remain in generations.json and the native traces; duplicates were intentional cross-checks or part of the regular grid.

| Gen | Trial | William cost/flips/roughness | Public cost/flips/roughness | Assessment |
|---:|---|---|---|---|
| 0 | model=stock | 0.2521/1.0100/0.0000 | 0.6110/0.7200/0.0000 | Stock reference |
| 1 | model=v1 | 0.0710/2.2400/0.0000 | 0.2671/1.6600/0.0629 | V1 reference; tracking improves, reversals alone not failure |
| 2 | model=v1, error_gain=0.15, friction=0 | 0.1891/1.7700/0.0000 | 0.7829/1.8300/0.0000 | Tracking loses useful restoring action, especially public plant |
| 3 | model=v1, error_gain=0.15, friction=0.5 | 0.1685/1.8000/0.0000 | 0.7278/1.6600/0.0000 | Tracking loses useful restoring action, especially public plant |
| 4 | model=v1, error_gain=0.15 | 0.1523/1.7600/0.0000 | 0.7331/1.6700/0.0000 | Tracking loses useful restoring action, especially public plant |
| 5 | model=v1, error_gain=0.3, friction=0 | 0.1771/1.8500/0.0000 | 0.7402/1.8200/0.0000 | Tracking loses useful restoring action, especially public plant |
| 6 | model=v1, error_gain=0.3, friction=0.5 | 0.1444/1.8600/0.0000 | 0.6740/1.6800/0.0000 | Tracking loses useful restoring action, especially public plant |
| 7 | model=v1, error_gain=0.3 | 0.1212/1.9800/0.0000 | 0.6068/1.4800/0.0000 | Tracking loses useful restoring action, especially public plant |
| 8 | model=v1, error_gain=0.5, friction=0 | 0.1619/1.7500/0.0000 | 0.7594/1.8100/0.0000 | Tracking loses useful restoring action, especially public plant |
| 9 | model=v1, error_gain=0.5, friction=0.5 | 0.1202/1.9700/0.0000 | 0.6016/1.5500/0.0000 | Tracking loses useful restoring action, especially public plant |
| 10 | model=v1, error_gain=0.5 | 0.0972/2.0700/0.0000 | 0.4637/1.5700/0.0000 | Tracking loses useful restoring action, especially public plant |
| 11 | model=v1, error_gain=0.7, friction=0 | 0.1510/1.7800/0.0000 | 0.7160/1.6700/0.0000 | Tracking loses useful restoring action, especially public plant |
| 12 | model=v1, error_gain=0.7, friction=0.5 | 0.1038/2.0300/0.0000 | 0.5147/1.6600/0.0000 | Retained sensitivity/Pareto alternative; not selected |
| 13 | model=v1, error_gain=0.7 | 0.0833/2.0900/0.0000 | 0.3692/1.6200/0.0000 | Retained sensitivity/Pareto alternative; not selected |
| 14 | model=v1, friction=0 | 0.1356/1.8300/0.0000 | 0.6353/1.7800/0.0000 | Tracking loses useful restoring action, especially public plant |
| 15 | model=v1, friction=0.5 | 0.0877/2.0300/0.0000 | 0.4078/1.6700/0.0000 | Retained sensitivity/Pareto alternative; not selected |
| 16 | model=v1 | 0.0710/2.2400/0.0000 | 0.2671/1.6600/0.0629 | V1 reference; tracking improves, reversals alone not failure |
| 17 | model=v1, error_gain=1.4, friction=0 | 0.1189/1.9400/0.0000 | 0.5918/1.6000/0.0000 | Tracking loses useful restoring action, especially public plant |
| 18 | model=v1, error_gain=1.4, friction=0.5 | 0.0754/2.1900/0.0000 | 0.3084/1.5500/0.0000 | Retained sensitivity/Pareto alternative; not selected |
| 19 | model=v1, error_gain=1.4 | 0.0642/2.6500/0.0000 | 0.1935/1.4900/0.0894 | Retained sensitivity/Pareto alternative; not selected |
| 20 | model=v1, error_gain=0.5, ff_tau=0.03, friction=0 | 0.1732/0.5300/0.0000 | 0.7402/0.5600/0.0000 | Tracking loses useful restoring action, especially public plant |
| 21 | model=v1, error_gain=0.5, ff_tau=0.08, friction=0 | 0.1945/0.2300/0.0000 | 0.7730/0.1800/0.0000 | Tracking loses useful restoring action, especially public plant |
| 22 | model=v1, error_gain=0.5, ff_tau=0.15, friction=0 | 0.2206/0.0700/0.0000 | 0.8209/0.0900/0.0000 | Tracking loses useful restoring action, especially public plant |
| 23 | model=v1, error_gain=0.5, ff_tau=0.3, friction=0 | 0.2741/0.0400/0.0000 | 0.9198/0.0300/0.0000 | Tracking loses useful restoring action, especially public plant |
| 24 | model=v1, error_gain=0.5, friction=0, jerk_scale=0 | 0.2768/2.0700/0.0000 | 0.8372/1.9800/0.0000 | Tracking loses useful restoring action, especially public plant |
| 25 | model=v1, error_gain=0.5, jerk_scale=0.25, friction=0 | 0.2308/2.0400/0.0000 | 0.8178/1.8900/0.0000 | Tracking loses useful restoring action, especially public plant |
| 26 | model=v1, error_gain=0.5, jerk_scale=0.5, friction=0 | 0.1988/1.9200/0.0000 | 0.7930/1.7800/0.0000 | Tracking loses useful restoring action, especially public plant |
| 27 | model=v1, error_gain=0.5, slew=0.3, friction=0 | 0.2611/0.0000/0.0000 | 0.8517/0.0000/0.0000 | Tracking loses useful restoring action, especially public plant |
| 28 | model=v1, error_gain=0.5, slew=0.6, friction=0 | 0.2007/0.0000/0.0000 | 0.7616/0.0000/0.0000 | Tracking loses useful restoring action, especially public plant |
| 29 | model=v1, error_gain=0.5, friction=0 | 0.1824/0.0100/0.0000 | 0.7436/0.0100/0.0000 | Tracking loses useful restoring action, especially public plant |
| 30 | model=v1, error_gain=0.5, slew=1.5, friction=0 | 0.1720/1.3000/0.0000 | 0.7482/1.2900/0.0000 | Tracking loses useful restoring action, especially public plant |
| 31 | model=v1, error_tau=0.02 | 0.0716/1.8700/0.0000 | 0.2697/1.2900/0.0881 | Retained sensitivity/Pareto alternative; not selected |
| 32 | model=v1, friction_tau=0.02 | 0.0718/2.0100/0.0000 | 0.2683/1.2800/0.0831 | Retained sensitivity/Pareto alternative; not selected |
| 33 | model=v1, error_tau=0.05 | 0.0747/1.6800/0.0000 | 0.2755/1.2800/0.1152 | Retained sensitivity/Pareto alternative; not selected |
| 34 | model=v1, friction_tau=0.05 | 0.0738/1.8900/0.0000 | 0.2749/1.2700/0.1034 | Retained sensitivity/Pareto alternative; not selected |
| 35 | model=v1, error_tau=0.1 | 0.0784/1.5700/0.0000 | 0.2861/1.1800/0.1232 | Retained sensitivity/Pareto alternative; not selected |
| 36 | model=v1, friction_tau=0.1 | 0.0768/1.7800/0.0000 | 0.2847/1.2300/0.1122 | Retained sensitivity/Pareto alternative; not selected |
| 37 | model=v1, error_tau=0.2 | 0.0842/1.5400/0.0000 | 0.3111/1.2200/0.0417 | Filtering adds response lag; no clear Pareto win |
| 38 | model=v1, friction_tau=0.2 | 0.0799/1.7100/0.0000 | 0.3041/1.2000/0.0659 | Filtering adds response lag; no clear Pareto win |
| 39 | model=v1, error_tau=0.4 | 0.0919/1.5200/0.0000 | 0.3569/0.9900/0.0590 | Filtering adds response lag; no clear Pareto win |
| 40 | model=v1, friction_tau=0.4 | 0.0849/1.8200/0.0000 | 0.3403/0.9300/0.0000 | Filtering adds response lag; no clear Pareto win |
| 41 | model=v1, slew=0.2 | 0.4738/0.0000/0.0000 | 1.3876/0.0000/0.0000 | Less command activity but added tracking lag |
| 42 | model=v1, slew=0.3 | 0.1901/0.0000/0.0000 | 0.5318/0.0000/0.0890 | Less command activity but added tracking lag |
| 43 | model=v1, slew=0.5 | 0.1188/0.0000/0.0000 | 0.3461/0.0000/0.0000 | Less command activity but added tracking lag |
| 44 | model=v1, slew=0.8 | 0.1008/0.0000/0.0000 | 0.3077/0.0000/0.0000 | Less command activity but added tracking lag |
| 45 | model=v1 | 0.0994/0.0000/0.0000 | 0.3013/0.0000/0.0000 | Less command activity but added tracking lag |
| 46 | model=v1, slew=1.5 | 0.0915/1.6600/0.0000 | 0.2827/1.1100/0.0000 | Less command activity but added tracking lag |
| 47 | model=v1, slew=2 | 0.0872/1.7700/0.0000 | 0.2823/1.2800/0.0000 | Less command activity but added tracking lag |
| 48 | model=v1, slew=3 | 0.0717/1.9200/0.0000 | 0.2652/1.3600/0.0712 | Retained sensitivity/Pareto alternative; not selected |
| 49 | model=v1, error_gain=1.4, error_tau=0.05 | 0.0705/1.9400/0.0426 | 0.2038/1.1200/0.1494 | S4 selected: stronger feedback with causal damping |
| 50 | model=v1, error_gain=1.4, error_tau=0.15 | 0.0759/1.7100/0.0000 | 0.2237/0.8800/0.1755 | Filtering adds response lag; no clear Pareto win |
| 51 | model=v1, error_gain=2, error_tau=0.05 | 0.0741/2.3000/0.1425 | 0.1670/1.1900/0.2672 | S5 selected: stronger straight tracking; inspect output tradeoff |
| 52 | model=v1, error_gain=2, error_tau=0.15 | 0.0842/1.6200/0.1672 | 0.1810/0.9500/0.2520 | Filtering adds response lag; no clear Pareto win |
| 53 | model=v1, error_gain=3, error_tau=0.05 | 0.1033/2.4500/0.3225 | 0.2043/1.3700/0.5315 | Straight error improves; total error/output energy tradeoff |
| 54 | model=v1, error_gain=3, error_tau=0.15 | 0.1187/1.8200/0.3808 | 0.1984/1.0300/0.5083 | Straight error improves; total error/output energy tradeoff |
| 55 | model=v1, ff_tau=0.02 | 0.0717/1.2000/0.0000 | 0.2665/0.8500/0.0536 | S3 selected: preserves tracking; less command ripple |
| 56 | model=v1, ff_tau=0.05 | 0.0768/0.9300/0.0000 | 0.2747/0.7200/0.0802 | Retained sensitivity/Pareto alternative; not selected |
| 57 | model=v1, ff_tau=0.1 | 0.0836/0.8800/0.0000 | 0.2876/0.5200/0.0575 | Retained sensitivity/Pareto alternative; not selected |
| 58 | model=v1, ff_tau=0.2 | 0.0937/0.8400/0.0000 | 0.3085/0.5100/0.0000 | Retained sensitivity/Pareto alternative; not selected |
| 59 | model=v1, error_gain=1.4, ff_tau=0.02 | 0.0645/1.5100/0.0000 | 0.1960/0.9100/0.0994 | Retained sensitivity/Pareto alternative; not selected |
| 60 | model=v1, error_gain=1.4, ff_tau=0.05 | 0.0670/1.2900/0.0000 | 0.1993/0.8100/0.1212 | Retained sensitivity/Pareto alternative; not selected |
| 61 | model=v1, error_gain=1.4, ff_tau=0.1 | 0.0723/1.1400/0.0000 | 0.2095/0.6800/0.1176 | Retained sensitivity/Pareto alternative; not selected |
| 62 | model=v1, error_gain=1.4, ff_tau=0.2 | 0.0775/1.0800/0.0000 | 0.2208/0.5600/0.0569 | Retained sensitivity/Pareto alternative; not selected |
| 63 | model=v1, error_gain=1.11, friction=0.904, ff_tau=0.0309, error_tau=0.0994 | 0.0826/0.7100/0.0000 | 0.2796/0.5200/0.1559 | Retained sensitivity/Pareto alternative; not selected |
| 64 | model=v1, error_gain=1.6, friction=0.918, ff_tau=0.0547, error_tau=0.0881 | 0.0819/0.7500/0.0861 | 0.2107/0.4500/0.2058 | Retained sensitivity/Pareto alternative; not selected |
| 65 | model=v1, error_gain=1.11, friction=0.786, ff_tau=0.0386, error_tau=0.00556 | 0.0774/0.8800/0.0000 | 0.2942/0.7300/0.0720 | Retained sensitivity/Pareto alternative; not selected |
| 66 | model=v1, error_gain=1.68, friction=0.876, ff_tau=0.0297, error_tau=0.0834 | 0.0787/0.9300/0.0846 | 0.2036/0.5500/0.2000 | Retained sensitivity/Pareto alternative; not selected |
| 67 | model=v1, error_gain=1.39, friction=0.922, ff_tau=0.0401, error_tau=0.0974 | 0.0804/0.7600/0.0386 | 0.2298/0.4900/0.1810 | Retained sensitivity/Pareto alternative; not selected |
| 68 | model=v1, error_gain=1.01, friction=0.826, ff_tau=0.0419, error_tau=0.0263 | 0.0814/0.7300/0.0000 | 0.3123/0.5500/0.0852 | Retained sensitivity/Pareto alternative; not selected |
| 69 | model=v1, error_gain=0.925, friction=0.845, ff_tau=0.027, error_tau=0.0445 | 0.0826/0.7900/0.0000 | 0.3347/0.5700/0.0767 | Retained sensitivity/Pareto alternative; not selected |
| 70 | model=v1, error_gain=1.46, friction=0.87, ff_tau=0.0263, error_tau=0.0122 | 0.0673/1.2500/0.0000 | 0.2069/0.7600/0.1240 | Retained sensitivity/Pareto alternative; not selected |
| 71 | model=v1, error_gain=1.46, friction=0.646, ff_tau=0.0525, error_tau=0.0341 | 0.0779/0.6900/0.0000 | 0.2625/0.5300/0.1289 | Retained sensitivity/Pareto alternative; not selected |
| 72 | model=v1, error_gain=1.7, friction=0.739, ff_tau=0.000472, error_tau=0.0212 | 0.0661/2.0500/0.0000 | 0.2019/1.1100/0.1286 | Retained sensitivity/Pareto alternative; not selected |
| 73 | model=v1, error_gain=1.16, friction=0.953, ff_tau=0.0496, error_tau=0.0293 | 0.0757/0.7800/0.0000 | 0.2457/0.5100/0.1255 | Retained sensitivity/Pareto alternative; not selected |
| 74 | model=v1, error_gain=1.74, friction=0.748, ff_tau=0.0402, error_tau=0.0596 | 0.0765/0.9400/0.0530 | 0.2099/0.6100/0.1833 | Retained sensitivity/Pareto alternative; not selected |
| 75 | model=v1, error_gain=1.61, friction=0.824, ff_tau=0.00707, error_tau=0.0797 | 0.0747/1.1900/0.0539 | 0.2119/0.8400/0.1725 | Retained sensitivity/Pareto alternative; not selected |
| 76 | model=v1, error_gain=1.33, friction=0.898, ff_tau=0.0546, error_tau=0.0146 | 0.0719/0.9300/0.0000 | 0.2265/0.6900/0.1158 | Retained sensitivity/Pareto alternative; not selected |
| 77 | model=v1, error_gain=1.51, friction=0.868, ff_tau=0.0644, error_tau=0.0916 | 0.0827/0.6500/0.0639 | 0.2277/0.3700/0.1898 | Retained sensitivity/Pareto alternative; not selected |
| 78 | model=v1, error_gain=1.07, friction=0.95, ff_tau=0.0564, error_tau=0.0694 | 0.0841/0.6300/0.0000 | 0.2808/0.4500/0.1563 | Retained sensitivity/Pareto alternative; not selected |
| 79 | model=v1, error_gain=1.54, friction=0.859, ff_tau=0.0236, error_tau=0.0766 | 0.0763/1.0200/0.0550 | 0.2147/0.6800/0.1827 | Retained sensitivity/Pareto alternative; not selected |
| 80 | model=v1, error_gain=1.4, friction=0.907, ff_tau=0.0775, error_tau=0.0768 | 0.0827/0.6200/0.0290 | 0.2354/0.3700/0.1890 | Retained sensitivity/Pareto alternative; not selected |
| 81 | model=v1, error_gain=0.999, friction=0.763, ff_tau=0.0296, error_tau=0.0917 | 0.0880/0.6700/0.0000 | 0.3444/0.4600/0.0856 | Retained sensitivity/Pareto alternative; not selected |
| 82 | model=v1, error_gain=1.23, friction=0.819, ff_tau=0.00223, error_tau=0.0192 | 0.0703/1.8300/0.0000 | 0.2552/1.1400/0.0992 | Retained sensitivity/Pareto alternative; not selected |
| 83 | model=v1, error_gain=0.819, friction=0.823, ff_tau=0.054, error_tau=0.093 | 0.0995/0.3900/0.0000 | 0.3873/0.2300/0.0000 | Retained sensitivity/Pareto alternative; not selected |
| 84 | model=v1, error_gain=1.34, friction=0.784, ff_tau=0.0278, error_tau=0.0537 | 0.0759/0.9200/0.0000 | 0.2477/0.6100/0.1476 | Retained sensitivity/Pareto alternative; not selected |
| 85 | model=v1, error_gain=1.58, friction=0.972, ff_tau=0.0036, error_tau=0.0824 | 0.0745/1.4600/0.0873 | 0.1984/0.8800/0.1886 | Retained sensitivity/Pareto alternative; not selected |
| 86 | model=v1, error_gain=1.62, friction=0.977, ff_tau=0.0643, error_tau=0.0142 | 0.0684/1.1300/0.0173 | 0.1827/0.6800/0.1586 | Retained sensitivity/Pareto alternative; not selected |
| 87 | model=v2 | 0.0869/2.4300/0.0000 | 0.2296/1.7900/0.0000 | Untuned V2 causal-history reference; offline only |

## Root causes and what failed

V2 is not rejected by the perceptual screen: it has lower wheel/body output roughness than V1 on William’s plant with similar tracking. It remains offline because its 26-input contract is not accepted by the runtime override, and its reused history adapter supplies a pre-NNLC controller-output proxy rather than applied-CAN history. In fact LatControlTorqueExt assigns the current base-controller output before CandidateModel.update runs; calling that actual past applied torque would be wrong. A causal applied-command-history integration and retraining/evaluation against that exact contract is the next V2 implementation step. S3 is recommended for the first bounded, reviewable 18-input integration, not because V2 has too many reversals.

- Simply lowering gain or removing friction makes commands calmer while losing tracking. EPS stiction requires a restoring action; penalizing raw torque motion rewards a controller that does too little.
- NNLC mixes learned feedforward, learned setpoint-minus-measurement feedback, high-accel error blending, explicit friction override and torque-space PID. The inverse trainer’s third feature was achieved jerk, but runtime uses friction/error proxies; the mismatch is real. Zeroing/scaling that input alone did not preserve robust tracking. No claim that a retrained model has identified a causal EPS law.
- Filtering the complete error or friction response is causal but adds feedback lag. Slow output slew is especially costly to responsiveness. Short FF-only filtering removes frame-to-frame feedforward activity while retaining the restoring path; it is shaping at the identified path, not a broad final-actuator low-pass.
- Stronger error gain with short error filtering gives better small-plan tracking than V1. The cost is output energy on some windows. Those candidates remain live alternatives rather than being rejected over command flips.
- No new temporal-consistency supervised fit, differentiable-plant policy, full RL, MPC, or 250 h retraining was completed. They are not recorded as failed experiments. These 88 controller trials test the more direct closed-loop-objective path first.

## Runtime integration, checks and artifacts

Only shared runtime edit: openpilot/sunnypilot/selfdrive/controls/lib/nnlc/nnlc.py. Added NnlcResponseShaper, validated JSON metadata loaded only for an explicit model override, causal separate feedback/FF filtering, A/B and idle/driver/low-speed state resets. No new param. Default stock path and existing metadata-free V1 remain identical. Invalid metadata restores stock model; SafeNNTorqueModel retains finite bounded inference and sanitized-stock fallback on invalid live values. Existing PID limits bound complete command. The numerical shaper guard resets invalid state. No OdysseySteer edit or other worker directory edit.

Tests: the new causal step-response/reset seam first failed collection because the runtime class did not exist, then passed. Existing NNLC tests plus four new tests pass: 39 total. Tests cover metadata absence/invalid values, bad-input reset, causal filtering and actual override construction with stock identity. Ruff passes on this task’s Python files and nnlc.py. Full production construction was exercised by calib.evaluate --controller ...:runtime_factory; its output is scratchpad/calib/nnlc_smooth_runtime.json. That evaluator’s overall lane calibration gate remains false by design; candidate tracking is not a baseline-calibration test.

| Model | Host median ms | Host p95 ms | Observed host max ms |
|---|---:|---:|---:|
| V1 | 0.0863 | 0.0891 | 0.1693 |
| S3 | 0.0873 | 0.0907 | 0.1728 |
| S4 | 0.0887 | 0.0921 | 0.1803 |
| S5 | 0.0877 | 0.0906 | 0.1854 |

Timing measures complete high-accel controller updates (2,000 measured frames after 200 warmup) on Mac Studio. A 5–10× hardware slowdown implies roughly 0.44–0.89 ms median, consistent with the requested ~2 ms target. This is not a Snapdragon deadline guarantee; no device benchmark was authorized or attempted.

New source: tools/odyssey_sim/nnlc_smooth/{experiment.py,score.py,replay.py,benchmark.py,report.py,test_runtime.py,README.md}. Candidate JSONs are untracked inside the neural_network_data submodule. Hashes:
- `HONDA_ODYSSEY_WILLIAM_S3.json` SHA256 `869c050c4904937f4749161b977b36c45ac3393b979d19d3ed26db1c5f680ac4`
- `HONDA_ODYSSEY_WILLIAM_S4.json` SHA256 `db0522d603cb67b2d49832830c746fc072020793de002c10963c88cd2b592ede`
- `HONDA_ODYSSEY_WILLIAM_S5.json` SHA256 `461bda7b7d4fd395dcf342868c56f9e91e0c756f861b8db6f42672280393372f`

Perception source SHA256 `d24f8eb9dd8c5c3652ef05adbe8c6bf825d1054cc4c4f3cb4664384c051d9161`.

Artifacts: scratchpad/nnlc_smooth/{generations.json,selection_scored.json,test.json,test_scored.json,selected.json,replay.json,benchmark.json} and all native NPZ traces; logs copied alongside them. The README contains reproduction commands and the explicit default-off model selection. All existing branch/untracked work is preserved; nothing staged.

## Remaining risks and next concrete step

The plant is identified at 10 Hz; high-frequency wheel/hand perception is not independently validated. No calibrated hand-torque response is available; no hands-on detection threshold for William is known. First-order plant motion and quantized feedback can suppress real EPS oscillation modes. Original selection angle conversion uses CarParams defaults; final test traces use per-frame steer ratio/stiffness for wheel-motion reconstruction. Public body dynamics and physical parameter uncertainty are sensitivity checks, not confidence bounds. Recorded plans and closed-loop historical data do not identify counterfactual planner/lane behaviour.

The lead can package S3 and S5 with the bounded hook, then combine the inner-loop candidates with the wander/calib3 worker’s validated lane-loop slice and the low-speed/curves workers’ separate add-ons. Do not silently stack independent controller interventions: rerun the integrated candidate against stock with the same masks and complete wheel/body/hand coverage. Any vehicle rollout remains a separate approved lead action.

## Plain-language summary

The van-specific model follows the requested steering better in the calibrated simulator. Its extra electrical command reversals are not a sound reason to throw it away. A very short filter keeps that tracking gain and makes the command quieter; stronger-feedback versions follow straights still better. We have reviewable, default-off candidates and reproducible tests. We have not yet proved reduced real lane wandering, better parking-speed steering, earlier real curve entry or William-perceived hand smoothness. No vehicle settings changed.
