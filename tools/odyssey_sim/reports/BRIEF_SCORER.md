# Brief: extend the road A/B scorer to the locked forecast metrics

Repo `~/Code/odyssey-pilot`, branch `odyssey-sim`.

- **Write scope:** edit only `tools/odyssey_wander.py` and its tests (under `tools/tests/`, or add `tools/tests/test_odyssey_wander_ab.py`), plus scratch files.
- **Don't** commit, push, or touch the comma device.

## Task

The next road drive runs `NnlcModelAB`, which rotates stock NNLC, S3 and S5 in 180 s blocks on the monotonic clock. See `openpilot/sunnypilot/selfdrive/controls/lib/odyssey_ab.py`.

The simulator has already locked its predictions for that drive. They are in `tools/odyssey_sim/reports/vsim3/predictions.json`, with exact metric definitions in `tools/odyssey_sim/reports/vsim3/protocol.json`, and the shared implementations are in `tools/odyssey_sim/vsim/metrics.py`. Read `tools/odyssey_sim/reports/report_vsim3.md`, section 4.

`tools/odyssey_wander.py <route dir> --ab nnlc_model` scores only wander today. Extend its A/B report so it also gives, per arm, the other two forecast metrics:
- **Signed curve run-wide:** reuse the vsim metric definition.
- **Body lateral jerk:** the 20 Hz RMS derivative of vEgo × carControl.angularVelocity[2].

The extended report must:
- **match the scorer's existing rules:** same block assignment, same 30 s settling, same autonomous selection;
- **compare each candidate with stock:** report the candidate-minus-stock difference with a bootstrap interval over blocks, in the same units as `predictions.json`;
- **print the locked prediction alongside** each difference, and say whether the road result falls inside the predicted interval.

Use the definitions exactly as locked. Don't choose new ones.

## Checks

- **Unit tests:**
  - arm assignment and settling;
  - a synthetic route where the jerk and width differences are known;
  - the prediction file loads and prints.
- **A smoke run on an existing route.** Use `~/comma-logs/04b44e4c4fe44232_000001ab--9191c0d55d`. That drive had no A/B, so the arms are just time blocks; report it as a smoke test only.
- **Ruff** and `.venv/bin/python tools/test_runner.py tools/tests` both pass.

Finish with a short summary message: what changed, and the exact command to score the next drive.
