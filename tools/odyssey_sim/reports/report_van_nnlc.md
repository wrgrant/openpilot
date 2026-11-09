# Van-specific NNLC experiment — final implementation receipt

**Recommendation: retain stock V0 + HONDA_ODYSSEY NNLC as the default. V1 is an opt-in research candidate; do not deploy V2.** Both trained models reduce held-out inverse torque error, but closed-loop jerk and reversal regressions prevent a claim of better driving. No device was accessed, no branch switched, no commit or push made.

## Data and training

Public source: https://github.com/commaai/comma-steering-control. Reused the plant worker’s deterministic every-tenth-segment cache: 1,499 of 14,987 sixty-second CSVs (~25 hours). After filtering and every-other-frame sampling: 333,232 examples. Segment identities remain intact; random frames are used only for optimizer minibatches, never to split training/test. Community vehicle identity is unavailable.

William’s EPS is `39990-THR-A020` on all six included routes. Public sampled firmware counts: {"b'39990-THR-A020\\x00\\x00'": 1332, "b'39990-THR-A030\\x00\\x00'": 166, '': 1}. A020 samples carry weight 3, other firmware weight 1. Small |torque|<0.1 receives another factor 3. Both candidates pretrain on public data and adapt to William’s van. Public data was not expanded to all 250 hours: this already supplies >20x as many examples as the van, whose measured gain differs substantially; model selection on a complete owner route stops pretraining/adaptation when it ceases helping.

| Owner route | Role | Examples |
|---|---|---:|
| 0000018c--9a64dd12bc | adaptation | 5916 |
| 0000019b--67c64d0ac6 | adaptation | 1107 |
| 0000019c--0d91088e6e | adaptation | 7963 |
| 000001a7--a7f62f4b3f | weight selection | 2482 |
| 000001aa--ba236ee054 | final test | 4463 |
| 000001ab--9191c0d55d | final test | 4961 |

The available closed-loop caches cover 18c,19b,19c,a7,aa,ab. a3 and other routes were not added. Every owner response is recomputed from steering angle minus live angle offset, using per-frame steerRatio/stiffness and vehicleParameters.roll in VehicleModel. Labels are applied CAN STEER_TORQUE/4096 interpolated from the plant cache; this sign matches positive openpilot curvature. Public labels are negative steerFiltered. An initial exploratory fit used negative logged PID output; it was superseded after observing 0.01–0.05 mean command differences from applied CAN. All tables here use the corrected applied-CAN fit.

Both models: deterministic seed 20261109, CPU torch training in separate scratch nnlc-venv, Adam lr .002, 4096-ish minibatches, up to 180,000 weighted draws per epoch, 35 public epochs then 60 owner epochs, select the best weighted validation MSE on a7 throughout. Loss includes a penalty for an insufficient positive restoring gradient under simultaneous acceleration-input increases. Checkpoint selection never sees aa/ab. Hidden layers use sigmoid; output is linear. V1 18→64→32→1 (3,329 parameters), V2 26→64→32→1 (3,841 parameters). No torch enters the device runtime.

V1 preserves the original 18-input schema. V2 adds angle/rate, current acceleration, acceleration 0.1/0.3/0.5 seconds ago, and applied torque 0.3/0.5 seconds ago. Its offline closed-loop adapter substitutes pre-NNLC controller-output history for unavailable applied CAN history. This approximation and autoregressive persistence are limitations, not evidence of a learned EPS law.

The inverse task uses achieved steering-angle acceleration 0.32 s after command time, plus its -0.3/-0.2/-0.1/+0.3/+0.6/+1.0/+1.5 context. Future achieved signals are permissible inverse identification targets but are NOT available to the deployed controller. Runtime NNLC substitutes the plan and desired history. NNLC’s third input is actually its friction proxy, while the identification task uses acceleration derivative. Setpoint/measurement calls additionally repeat the instantaneous acceleration with LOW_SPEED curvature augmentation. These distribution differences are material. The fitted 0.87 s time constant is not a pure delay; it was not substituted for the 0.32 s alignment.

## Held-out inverse torque prediction

Filtering excludes |applied torque| >= .98, speed <=1 m/s and acceleration magnitude >=4 m/s²; it requires active/no-driver intervention throughout each context window. Thus the error tables characterize nonsaturated inverse behavior. Only 52 test examples are below 10 mph; saturation performance is not established.

Torque is normalized; MAE 0.05 means five percent of full command. Metrics below pool aa/ab by sample count. No held-out frames were used to tune weights. All results are against applied CAN torque; they are not trajectory-following metrics.

| Band | n | Stock MAE / RMSE | V1 MAE / RMSE | V2 MAE / RMSE |
|---|---:|---:|---:|---:|
| all | 9424 | 0.0910 / 0.1262 | 0.0810 / 0.1136 | 0.0572 / 0.0800 |
| small_torque | 4282 | 0.0670 / 0.0943 | 0.0448 / 0.0556 | 0.0388 / 0.0505 |
| large_torque | 5142 | 0.1110 / 0.1475 | 0.1110 / 0.1452 | 0.0726 / 0.0981 |
| 0-10mph | 52 | 0.3704 / 0.4607 | 0.3572 / 0.4316 | 0.2288 / 0.2892 |
| 10-20mph | 235 | 0.1632 / 0.2349 | 0.1628 / 0.2263 | 0.1233 / 0.1722 |
| 20-35mph | 1778 | 0.1099 / 0.1480 | 0.1047 / 0.1429 | 0.0793 / 0.1055 |
| 35-50mph | 4675 | 0.0889 / 0.1180 | 0.0781 / 0.1028 | 0.0535 / 0.0691 |
| 50+ mph | 2684 | 0.0705 / 0.0906 | 0.0576 / 0.0725 | 0.0401 / 0.0509 |

| Route / slice | Stock MAE | V1 MAE | V2 MAE |
|---|---:|---:|---:|
| 000001aa / all | 0.0916 | 0.0814 | 0.0564 |
| 000001aa / small_torque | 0.0685 | 0.0445 | 0.0392 |
| 000001ab / all | 0.0905 | 0.0806 | 0.0580 |
| 000001ab / small_torque | 0.0656 | 0.0451 | 0.0385 |

## Closed loop on worker 2’s fitted plant

Reused worker 3’s windows, Controller interface, 100 Hz updates, plant adapter and Controls Challenge cost convention. Explicitly replaced its V1 core with the van’s V0 class for ALL arms. V1 uses the actual bounded runtime evaluator. No low-speed-angle/deadband post-step was enabled. Both trained models pass through the existing NNLC friction override and torque-space PID. Fixed .32 s controller delay and logged modelV2 plans; learned plant gain/friction/lag. aa/ab are excluded from plant fitting. Costs use 50×lataccel_cost + jerk_cost; lower is better.

Cells are **cost / acceleration jerk-cost / native 100 Hz torque reversals per second**. Each row is one worker-selected 30 s maximum-curvature window with 10 s warmup and a speed-band mask. Some bands have only a few scored seconds. No Honda actuator rate limiter or closed planner/lane feedback is simulated; this is a rejection screen, not proof of a road win. Plant validity begins at 5 m/s (~11 mph), so 0–10 mph rows are explicitly extrapolations and cannot support low-speed claims.

| Route / mph | scored n (10 Hz) | Stock | V1 | V2 |
|---|---:|---:|---:|---:|
| 000001aa / 0-10 | 36 | 1914.24 / 0.92 / 6.39 | 3.87 / 3.74 / 20.83 | 4.40 / 4.26 / 21.39 |
| 000001aa / 10-20 | 30 | 949.00 / 0.62 / 5.33 | 949.12 / 2.26 / 0.67 | 949.21 / 2.15 / 0.33 |
| 000001aa / 20-35 | 32 | 15.27 / 1.17 / 0.62 | 10.40 / 1.10 / 0.31 | 9.98 / 1.47 / 3.75 |
| 000001aa / 35-50 | 45 | 29.02 / 3.07 / 0.00 | 17.51 / 4.26 / 0.00 | 14.37 / 4.78 / 0.89 |
| 000001aa / 50-200 | 200 | 35.46 / 2.36 / 1.20 | 10.77 / 4.79 / 1.95 | 12.04 / 4.34 / 2.00 |
| 000001ab / 0-10 | 38 | 810.23 / 2.61 / 13.68 | 46.58 / 3.44 / 15.26 | 47.81 / 4.14 / 15.00 |
| 000001ab / 10-20 | 42 | 90.07 / 17.13 / 9.52 | 97.78 / 25.21 / 6.90 | 109.69 / 34.44 / 5.48 |
| 000001ab / 20-35 | 33 | 2476.73 / 1.05 / 0.61 | 2470.65 / 1.12 / 0.00 | 2469.66 / 1.31 / 0.61 |
| 000001ab / 35-50 | 168 | 15.03 / 1.16 / 0.89 | 9.58 / 1.78 / 1.25 | 7.95 / 1.98 / 1.43 |
| 000001ab / 50-200 | 42 | 9.97 / 0.48 / 3.57 | 6.83 / 2.17 / 2.86 | 6.39 / 0.96 / 3.33 |

V2 does not consistently beat V1, and both can increase jerk/reversals against stock. It is therefore retained only as a reproducible offline research artifact; there is no extended-model runtime toggle to accidentally enable it. The generic tinyphysics ONNX plant was not rerun for these candidates (onnxruntime is absent in the repo venv); the Odyssey-specific fitted plant was run. Worker 3’s existing generic comparison is separate evidence, not a result for these candidates.

## Logged-motion replay

Reused worker 1’s validated replay adapter, V0, original timing/lag and vehicleParameters. Bounded V1 vs stock on the first 20,000 control frames of each final-test route. Params writes are synchronous in the isolated replay namespace. MAE is agreement with logged PID output, not evidence of better steering; motion remains logged/open-loop.

| Route / mph | n | Stock command MAE / flips per s | V1 command MAE / flips per s |
|---|---:|---:|---:|
| 000001aa / 20-35 | 115 | 0.0000 / 5.26 | 0.0554 / 3.51 |
| 000001ab / 20-35 | 28 | 0.0000 / 0.00 | 0.1362 / 0.00 |
| 000001ab / 35-50 | 4590 | 0.0000 / 2.59 | 0.1377 / 2.75 |
| 000001ab / 50-999 | 6697 | 0.0001 / 1.91 | 0.1182 / 2.81 |

## Timing

Full controller updates at high acceleration (including error blending and up to five model evaluations), 2000 iterations after warmup, Mac Studio; not on-device measurements.

| Model | Mac median / p95 ms | 3X estimated median ms (5–10× slower) |
|---|---:|---:|
| stock | 0.0653 / 0.0756 | 0.327–0.653 |
| v1 | 0.0765 / 0.0884 | 0.382–0.765 |
| v2 | 0.0750 / 0.0887 | 0.375–0.750 |

This fits the requested ~2 ms estimate, including the complete controller update. It is not a certified worst-case deadline on Snapdragon hardware. No device benchmark was performed.

## Implementation, checks, and enabling

Uncommitted changes include an ODYSSEY.md link to the experiment guide. In the root checkout: `openpilot/common/params_keys.h` (persistent/backed-up string NnlcModelOverride), `nnlc/helpers.py` (validated selection, exclude candidate names from automatic matching), `nnlc/nnlc.py` (shared extension selection for V0 and V1), new `nnlc/safe_model.py` (bounded experimental inference and sanitized-stock fallback), new `nnlc/tests/test_model_override.py`, and the new `tools/odyssey_sim/nnlc_train/` scripts/README. Candidate JSONs are untracked inside the neural_network_data submodule, so the parent status shows that submodule dirty; a lead must package those artifacts explicitly. Existing stock JSON is unchanged.

Candidate hashes:
- `HONDA_ODYSSEY_WILLIAM_V1.json` SHA256 `625bc6122fd545800902b668fdcfe04835303b6fb1d562a888ce0d5cd370c814`
- `HONDA_ODYSSEY_WILLIAM_V2.json` SHA256 `56a390fce65b5e07505b7c1142415e01f72f0142fddbc4affe64dac4e8a2cc78`

Checks: params rebuilt locally with the requested scons target (build log nnlc_params_build.log); all 35 NNLC tests pass (nnlc_tests.log); targeted ruff passes. Tests cover unchanged default matching, absent/missing/path-traversal/unsupported overrides, real Params selection, finite bounded outputs including NaN/Inf fallback, monotonic coordinated acceleration sweeps across speed/roll, and selection through both controller classes. The override tests were observed failing before implementation, then passing. No subagents were spawned because this task is itself an implementation-subagent assignment; implementation self-review covered data/signs, default-off isolation, then runtime bounds and reproducibility.

To stage V1 on a reviewed build, while offroad set `NnlcModelOverride` to `HONDA_ODYSSEY_WILLIAM_V1.json`, keep `NeuralNetworkLateralControl=1`, disable `NnlcAB` for a fixed-model comparison, then restart controls/reboot. The new param is read at controller construction. Remove the override and restart to restore stock. See `tools/odyssey_sim/nnlc_train/README.md` for exact Params commands. No configuration was written to the van. V2 cannot be selected by this override (26 inputs are rejected).

Artifacts: nnlc_data_manifest.json, nnlc_train.log, nnlc_scores.json, nnlc_closedloop.json, nnlc_replay.json, nnlc_benchmark.json, nnlc_tests.log in this scratch directory. Public-only checkpoints also remain here, not in the shipped model directory.

## What William should take away

The van-specific models learned the small-command region better than the old community model. That alone does not solve the van: the controller can still chatter, saturation obscures inverse identification, future desired motion differs from future achieved motion, and the planner’s slow curve-exit request remains upstream. V1 is ready for lead review as an experiment; neither model has earned default deployment. The next concrete step is to fit a causal controller objective that penalizes command reversals and uses applied actuator history consistently, then verify through the validated V0 replay and rate-limited closed loop before any road comparison. Full 250-hour pretraining or a larger network is secondary to that objective/runtime alignment.
