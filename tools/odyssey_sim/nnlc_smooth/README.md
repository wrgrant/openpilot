# Closed-loop Odyssey NNLC shaping

Research on branch `odyssey-sim`; no device deployment. See the supplied scratch directory's `report_nnlc_smooth.md` for the complete generation log and limits.

`experiment.py` runs a deterministic 87-trial grid/random black-box search of the existing owner-trained V1 NNLC with causal controller shaping. Selection uses two eligible 60 s a7 windows on calibrated William plant v2 and public-plant sensitivity. Commands run at 100 Hz, logged plans at 10 Hz, with 10 s warmup, logged delay/tuning/pitch, quantized wheel feedback, and Honda actuator slew. The public sensitivity keeps its original schema-v1 dynamics; it is not a second calibrated v2 van.

`score.py` reconstructs unquantized plant motion, then calls the perception worker's `perceptible_roughness`. Test traces include live steer ratio/stiffness. Original selection traces use CarParams defaults for the conversion to wheel angle. The metric is a hypothesis with threshold sensitivity, not a probability of human detection. It cannot certify the 7–20 s wander goal; its first band starts at 0.1 Hz, and the lane loop remains uncalibrated.

Three candidates preserve V1's learned weights, with JSON `odyssey_smoothing` metadata:

- `HONDA_ODYSSEY_WILLIAM_S3.json`: 20 ms feedforward filter; conservative tracking-preserving candidate.
- `HONDA_ODYSSEY_WILLIAM_S4.json`: feedback ×1.4, 50 ms feedback filter; balanced research candidate.
- `HONDA_ODYSSEY_WILLIAM_S5.json`: feedback ×2, 50 ms feedback filter; straight-tracking candidate with more output energy.

The actual runtime hook in `nnlc/nnlc.py` reads this metadata only with an explicitly selected model override. No new params. Stock and V1 without metadata follow the original code path. Invalid metadata restores stock. SafeNNTorqueModel continues to bound inference and sanitize invalid live input. Shaping bypasses and resets with driver input or speed <5 m/s and resets after >100 ms between calls and on NNLC A/B switches. Runtime is NumPy plus the Python standard library.

From repo root:

```sh
PYTHONPATH=$PWD .venv/bin/python -m tools.odyssey_sim.nnlc_smooth.experiment
PYTHONPATH=$PWD .venv/bin/python -m tools.odyssey_sim.nnlc_smooth.score
PYTHONPATH=$PWD .venv/bin/python -m tools.odyssey_sim.nnlc_smooth.experiment --test
PYTHONPATH=$PWD .venv/bin/python -m tools.odyssey_sim.nnlc_smooth.score --test
PYTHONPATH=$PWD .venv/bin/python -m tools.odyssey_sim.nnlc_smooth.replay
PYTHONPATH=$PWD .venv/bin/python -m tools.odyssey_sim.nnlc_smooth.benchmark
PYTHONPATH=$PWD .venv/bin/python -m tools.odyssey_sim.calib.evaluate --logged-plan-only --controller tools.odyssey_sim.nnlc_smooth.experiment:runtime_factory --output nnlc_smooth_runtime.json
PYTHONPATH=$PWD .venv/bin/python tools/test_runner.py tools/odyssey_sim/nnlc_smooth/test_runtime.py openpilot/sunnypilot/selfdrive/controls/lib/nnlc/tests
.venv/bin/ruff check tools/odyssey_sim/nnlc_smooth openpilot/sunnypilot/selfdrive/controls/lib/nnlc/nnlc.py
```

All results/traces go under `scratchpad/nnlc_smooth`. `selected.json` freezes the three trials selected on a7 before aa/ab evaluation. `--start N` resumes an interrupted sweep, preserving earlier rows. Final evaluation never tunes against aa/ab. `NNLC_SMOOTH_MODEL` selects the model filename for `runtime_factory` and stays in the local isolated Params namespace. The report separates full-controller replay from motion prediction: replay agreement is not improved tracking.

After lead review, S3 is selected via the existing `NnlcModelOverride` filename param plus `NeuralNetworkLateralControl=1`, with `NnlcAB=0`, followed by a controls restart. Remove the override and restart for stock. No param or file on the vehicle was changed. Models are untracked artifacts inside the neural-network-data submodule and must be packaged explicitly by the lead.
