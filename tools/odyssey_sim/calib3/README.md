# Calibration pass 3

Offline research only. Read `scratchpad/report_calib3.md` for evidence and limitations. The independent lane closure and full behavior gates still fail; scores do not authorize rankings or deployment.

All writes are confined to this directory and `scratchpad/calib3/`. Source caches remain immutable. Route splits come from `calib.identify`. No device access, staging, commit or push is used.

## Reproduce

From the repo root, with `PYTHONPATH=$PWD`:

1. `.venv/bin/python -m tools.odyssey_sim.calib3.extract` preserves all model trajectory channels, six origin fits, latency/calibration and native control inputs. Existing complete caches are reused.
2. Use the supplied `scratchpad/wander/venv/bin/python` (scipy) for modules `audit`, `observe_fit`, `transport`, `spectral`, `fusion`, in that order. `audit` is the interior/filter sensitivity check. `spectral` is the pass-2-compatible definition and includes 30 s diagnostics and speed-band coverage. Fusion fits only training 30 s pieces.
3. `.venv/bin/python -m tools.odyssey_sim.calib3.reversals` runs recorded-motion bandwidth ablations. It clamps the integral to recorded values each step, an explicit forensic intervention; it is not a simulation. Earlier unclamped results are retained as `reversals_free_integrator.json`.
4. `.venv/bin/python -m tools.odyssey_sim.calib3.evaluate --output evaluation_g7.json` runs a complete paired behavior gate. The exact pass-2 controller factory contract is retained: `--model /absolute/18-input-model.json` or `--controller module:factory`. Factory receives `(metadata, Config)` and returns an adapter with `update(target_accel, measured_accel, state, future_plan)` and `measured_lataccel`. `--forcing transport` selects the earlier road reconstruction. `--logged-plan-only` restricts diagnostic scope. Outputs always go into `scratchpad/calib3/` (use a basename).
5. `.venv/bin/python -m unittest discover -s tools/odyssey_sim/calib3 -p 'test_*.py'`, then `.venv/bin/ruff check tools/odyssey_sim/calib3`.

## Importable APIs

- `geometry.origin`: offset, heading, curvature at x=0 in calibrated model coordinates. Calibration is already applied by modeld's image warp; do not transform the output twice.
- `observer.LaneObserver`: numpy-only causal camera/yaw filter with a colored curvature-bias state. `step(road_accel, body_accel, offset=None, offset_velocity=None, dt=.1)` can predict without camera updates. The dropout check fails; posterior camera agreement is not evidence of independent closure.
- `lane.records(forcing='fusion')`: pass-2 records with reconstructed origin geometry and camera-conditioned road acceleration. Fusion uses spatial curvature minus recorded body motion, temporal lane heading, and model lateral velocity, with training-only coefficients.
- `lane.consistent_future`: bounded local change to model future acceleration; retains recorded roll/pitch. This is an explicit approximate planner perturbation, not a regenerated modeld world prediction.

The road is reconstructed using recorded camera and body channels. It is not an independent surveyed forcing. The evaluator therefore retains `ranking_authorized=false` even if an individual behavior row passes. All inference with new controller motion remains conditional on the perception/planner surrogate. This prevents a candidate's apparently good score from overriding failed calibration.
