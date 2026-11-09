# William's Odyssey inverse models

**Research result: lower inverse prediction error does not establish better steering. Keep stock enabled by default.** V1 is a selectable experiment; V2 remains an offline candidate because the V0 closed-loop comparisons do not establish a smoothness win. See `ODYSSEY.md` for the actual on-car execution path.

The two candidate JSONs live in the `openpilot/sunnypilot/neural_network_data` submodule, beside `HONDA_ODYSSEY.json`. V1 is 18→64→32→1, sigmoid hidden layers; V2 is 26→64→32→1. Runtime uses numpy only. V2 adds angle, rate, current/100/300/500 ms acceleration history and 300/500 ms applied-command history. It is **not** accepted by the runtime override.

## Reproduce

From repo root, use repo Python and `PYTHONPATH=$PWD` for `prepare.py`, `closedloop.py`, `replay.py`, and `benchmark.py`. Use the separate scratch `nnlc-venv` Python (torch, numpy) for `train.py`. All scripts have main guards. Paths and scratch location are in `__init__.py`.

1. Other workers produce `scratchpad/plant_public.npz`, `plant_data/*.npz`, `closedloop_data/*.npz`, and `plant_model.json`. These are read-only inputs to this work.
2. `prepare.py`: 1-in-10 deterministic public segment sample, every other 10 Hz row; complete segment boundaries, active with no steering override, 1 s preceding washout and 2.2 s future eligibility. Apply the per-frame VehicleModel using `vehicleParameters` fields for owner acceleration. Owner target is applied CAN torque /4096 interpolated at sample time. Public target is negative `steerFiltered`. Positive output means positive openpilot acceleration.
3. `train.py`: firmware-matched public pretraining, then owner adaptation; train 18c/19b/19c, select weights using a7, final held-out evaluation aa/ab. Public weights: A020 3, others 1; small torque additionally weighted 3. Seed 20261109. Outputs include `nnlc_scores.json`.
4. `closedloop.py`: V0 + stock NNLC versus V1 versus offline V2, reusing worker 3's plant/rollout and maximum-curvature windows; records both 10 Hz and native 100 Hz reversals. `replay.py` reuses worker 1's validated logged-motion adapter for the first 20,000 control frames of aa/ab. Neither replay nor the fitted plant establishes road performance.
5. `benchmark.py`: complete controller update timing, including five NN evaluations at high acceleration, with the 5–10× requested slowdown estimate.

Training trajectory inputs are achieved acceleration aligned 320 ms ahead of the torque command, including its past/future context. At runtime NNLC uses desired/planned acceleration; its third input is a friction proxy (not simply physical jerk), and its setpoint/measurement evaluations use constant acceleration histories plus low-speed curvature augmentation. This mismatch, quantized angle, saturation, probes in the owner data and the unmodeled planner/lane feedback limit conclusions. The 0.87 s fitted plant time constant is not a pure delay and must not simply replace the 320 ms alignment.

## Opt-in selection (staged only)

After installing the reviewed source and model artifact and rebuilding params, while offroad:

```python
from openpilot.common.params import Params

p = Params()
p.put('NnlcModelOverride', 'HONDA_ODYSSEY_WILLIAM_V1.json', block=True)
p.put_bool('NeuralNetworkLateralControl', True, block=True)
```

Restart controls/reboot to load it. Remove `NnlcModelOverride` and restart to restore the default model. `NnlcAB` compares NNLC against non-NNLC; it does not rotate model files. Disable it for fixed-model comparisons. No device configuration was changed by these experiments.

Unset, missing, invalid or non-18-input overrides fall back to the original model. Override files must resolve directly inside the model directory. Candidate names containing `_WILLIAM_` are excluded from automatic fuzzy matching. Only explicitly selected models use the bounded evaluator, which clamps to [-1,1] and falls back to sanitized stock evaluation for invalid live inputs. Original stock inference remains unchanged.

Checks:

```sh
PATH=$PWD/.venv/bin:$PATH .venv/bin/scons -j8 openpilot/common/libparams_c.dylib
.venv/bin/python tools/test_runner.py openpilot/sunnypilot/selfdrive/controls/lib/nnlc/tests
.venv/bin/ruff check tools/odyssey_sim/nnlc_train openpilot/sunnypilot/selfdrive/controls/lib/nnlc
```
