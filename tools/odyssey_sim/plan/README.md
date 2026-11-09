# Plan-side Odyssey research

Offline experiments only. No device access, production edits, params registration,
commit or push. Use the existing `odyssey-sim` checkout and repository `.venv`.
The simulator is a perturbation model with failed held-out spectrum closure;
closed-loop differences cannot rank road performance.

## Run

Set `PLAN` to your permitted scratch directory. All evidence, scratch Params and
Ruff caches belong there. Disable Python bytecode output to preserve write scope.

```sh
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD"
.venv/bin/python -m tools.odyssey_sim.plan.catalog --scratch "$PLAN"
.venv/bin/python -m tools.odyssey_sim.plan.campaign --scratch "$PLAN" --split train
.venv/bin/python -m tools.odyssey_sim.plan.campaign --scratch "$PLAN" --split test
.venv/bin/python -m tools.odyssey_sim.plan.analyze --scratch "$PLAN"
.venv/bin/python -m tools.odyssey_sim.plan.summarize --scratch "$PLAN"
.venv/bin/python tools/test_runner.py tools/odyssey_sim/plan/test_compensation.py tools/odyssey_sim/plan/test_analysis.py tools/odyssey_sim/plan/test_models.py
.venv/bin/ruff check --cache-dir "$PLAN/ruff_cache" tools/odyssey_sim/plan
```

`campaign` imports the previous full native-eligibility/video inventory without
selecting on candidate outcomes. Stock means unwarped open-loop model replay;
closed-loop stock comparators are the unchanged `vsim3/*_stock` runs. Each
compensation gets one isolated process. Existing outputs are resumable; use a new
scratch directory for a changed generation. Failures remain explicit.

## Code map and contracts

- `compensation.py`: numpy-only public `Compensation.update` acceleration interface.
- `runner.py`: process-local vsim adapters. Compensation changes loop.target after
  stock action smoothing, never the model's recurrent state or previous action.
  `path` removes the logged near-lane 7–20s component throughout warmup and score,
  with derivative-consistent yaw. This uses future logs, and is an offline
  counterfactual only. `--refine prior.traces.json` applies one fixed harmonic
  offset correction throughout warmup and score. It does not prove independent road curvature or perfect lane
  holding. Check actual perceived residual and warp domain before interpreting it.
- `analyze.py`: injected-transfer decomposition, full covariance matrix, ideal
  acceleration-to-displacement equivalents, and tracking-gap equivalents. None
  is a causal percentage of real lane wander.
- `summarize.py`: matched model/plan scores, shared-reference pose transport and body/request curve timing.
- `sign_audit.py`: restores the negative-feedback sign before interpreting phase.
- `models.py`: default native ONNX plus an explicitly bounded legacy 20Hz ABI.
  Legacy uses 5 image frames sampled every four, 132 desire frames max-pooled by
  four, 128 feature frames sampled by four, and previous-output hidden state.
  This follows branch `compile_modeld.make_run_policy`/`stock_dependencies`.
- `catalog.py`: public catalog audit. Published source ONNX hashes in adjacent
  Hugging Face metadata can resolve through comma's public LFS bucket even where
  bundle artifacts themselves are GPU-compiled pickles. Hash-check every source.
- `model_campaign.py`: two alternate ONNX models, matched fixed train/held-out
  video subset, plus 7/10/15/20s injected training response. To execute it, place
  hash-verified `CHEDDAR.onnx` and `TSFDOM.onnx` in `$PLAN`, then run:
  `.venv/bin/python -m tools.odyssey_sim.plan.model_campaign --scratch "$PLAN" --all-windows`.
  Without `--all-windows`, the CLI uses the small documented discovery subset.

## Proposed integration, off by default

No production hook is installed. Lead would register `OdysseyPlanCompensation`
with false default, instantiate one compensator, and apply it between the
unaltered model action and `clip_curvature`. Supply real frame dt, explicit camera_age, fresh camera
confidence, lane origin and heading (positive vehicle-right), and every non-off
lane-change state. Low speed (<11.2m/s), stale/gapped/invalid camera data or low
confidence resets all filters and returns the original request immediately.
No dead reckoning through camera dropout. Caller must mark stale camera frames
invalid; calling repeatedly with an old high-confidence frame is not supported.
Correction is bounded to 0.15m/s². Startup after dropout has no filter history.

Lead is a modest fast-minus-slow additive band filter, not cancellation of 120°.
Lane mode adds `-.3*offset - 1.2*speed*heading`.
Smooth mode smooths the high-passed request: a residual **proxy**, not an
identified road/pose-independent residual. All gains are hypotheses frozen before
held-out scoring. Runtime benchmark is Mac evidence, not a measured comma timing.
No experiment here authorizes enabling these on the van.
