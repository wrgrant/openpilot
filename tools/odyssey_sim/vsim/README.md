# Video simulator research: real-model perturbation loop

This is a **gated research prototype**, not a controller-ranking simulator.
It runs the repository's actual default driving ONNX, both recorded cameras,
causal calibration/delay/traffic/desire context and the graph's recurrent state.
It applies a road-plane homography before camera preprocessing, in one resample.
The model predicts the plan and action; no fitted planner replaces it.

The moving 1a3 replay reaches desired-curvature r=0.99983. Dynamic, kinematically
consistent warps close the real-controller loop around a logged-trajectory
perturbation plant. Exhaustive controller comparisons and held-out curve/exit
measurements retain each window's pose departure, because close agreement can
carry recorded motion forward. **Controller rankings remain unvalidated**; see the
current research report for per-window gate outcomes and prospective forecasts.
The independent pulse gate is incomplete: 1a3 lacks the frozen no-pulse holds
needed for the requested train/validation experiment.

## Run

From `~/Code/odyssey-pilot` on `odyssey-sim`, choose where evidence files go:

```sh
export VSIM_SCRATCH=<a scratch folder for evidence files>
.venv/bin/python -m tools.odyssey_sim.vsim.run \
  --route ~/comma-logs/04b44e4c4fe44232_000001a3--c16cc8db23 \
  --controller logged --window 610:665 --output "$VSIM_SCRATCH/recheck.json"
```

The model runs on ONNX Runtime, installed in the repository's `.venv`
(`uv pip install --python .venv/bin/python onnxruntime==1.31.1`).
`--output` is optional; omit it to print without writing evidence files.
`--window start:end` is seconds from the first modelV2 output. Replay starts
15 seconds earlier for recurrence warmup; only >5m/s samples with >=200
continuous model frames count toward gate 1. Missing video is counted and
invalidates continuity. No model state from a video gap is treated as warmed.

Inject `--dy 0.1` (metres, positive right) or `--dpsi 0.5` (degrees,
positive right). The model result is counterfactual perception, not vehicle motion.
Custom bundles and nonzero CameraOffset are rejected until their exact adapters
exist. No silent default-model or calibration substitution occurs.

Output includes lane/path/curvature agreement, runtime breakdown and existing
`odyssey_wander` band/period metrics when >=45 continuous seconds are available.
These are unmasked perception diagnostics. Closed-loop runs additionally print
matched signed curve width, body release timing and body jerk in
`matched_curve_metrics`; takeover metrics remain unavailable.
Training closure exports pose and per-window wander RMS, period and spectrum.

## Code map

- `run.py`: causal input snapshots, encode frame pairing and open-loop gate CLI.
- `model.py`: ONNX Runtime CPU model with graph-owned state and native NV12 packing.
- `video.py`: upstream FrameReader index/cache with a decoder adapter for ffmpeg 8,
  which removed `-vsync`. Uses `-fps_mode passthrough`; Apple VideoToolbox on macOS.
- `geometry.py`: source-to-virtual homography and exact relative SE(2) integration.
  The projection uses the inverse homography to map virtual model pixels back
  to recorded camera pixels. Calibration is conjugated into sensor coordinates.
- `metrics.py`: existing 20Hz wander metric integration and explicit unavailable fields.
- `controls_audit.py`: independent 100Hz `clip_curvature` replay against controlsd.
- `plant_audit.py`: unchanged calibrated plant driven by recorded applied CAN command;
  reports each uninterrupted 60s window, alongside a logged-yaw integration oracle.

## Verify

```sh
.venv/bin/python tools/test_runner.py \
  tools/odyssey_sim/vsim/test_geometry.py tools/odyssey_sim/vsim/test_model.py \
  tools/odyssey_sim/vsim/test_perturbation.py tools/odyssey_sim/vsim/test_campaign.py
.venv/bin/ruff check tools/odyssey_sim/vsim
.venv/bin/python -m tools.odyssey_sim.vsim.plant_audit \
  --route 000001a3 --output "$VSIM_SCRATCH/plant_recheck.json"
.venv/bin/python -m tools.odyssey_sim.vsim.controls_audit \
  --route ~/comma-logs/04b44e4c4fe44232_000001a3--c16cc8db23 \
  --traces "$VSIM_SCRATCH/g3.traces.json" --output "$VSIM_SCRATCH/controls_recheck.json"
```

The existing `calib.identify.load` caches are read-only inputs for the plant audit.
The replay itself reads local rlogs and HEVC; it never accesses the comma.

## Interpretation and next step

A road-plane translation is exact for ground points under the pinhole camera
model. Rotation is depth-independent. Vertical objects, disocclusions, lens
model error and learned perception bias remain. Read `report_vsim.md` in the
parent scratch directory for every generation, measured gate failures,
source links, data coverage and the recommended next experiment.
Do not compensate heading by multiplying the injected yaw to force slope 1.
The closed loop uses `a_logged + P(u_sim) - P(u_logged)`, including the same
nonlinear deadband, friction, slew and lag in both plants. Matching commands cancel
algebraically; this is an invariant, not independent plant validation.


## Pass 2 commands and interpretation

Use the repository `.venv`; set a separate output directory:

```sh
export VSIM2="$VSIM_SCRATCH/../vsim2"
# Consistent perturbation; --dy is amplitude, yaw is dy derivative / logged speed.
.venv/bin/python -m tools.odyssey_sim.vsim.run \
  --route ~/comma-logs/04b44e4c4fe44232_000001a3--c16cc8db23 \
  --window 610:670 --dy .2 --shape sine --period 10 --output "$VSIM2/sine_10.json"
# Free run after 15s of recorded-state controller/model warmup.
.venv/bin/python -m tools.odyssey_sim.vsim.run \
  --route ~/comma-logs/04b44e4c4fe44232_000001a3--c16cc8db23 \
  --window 897:950 --controller tools.odyssey_sim.vsim.closed_loop:recorded \
  --scratch "$VSIM2" --output "$VSIM2/closure.json"
# Repeat free run with each of gain/friction/lag and .75/1.25.
# Add: --sensitivity gain --scale .75
.venv/bin/python -m tools.odyssey_sim.vsim.plant_audit \
  --route 000001a3 --native --output "$VSIM2/native_drift.json"
.venv/bin/python -m tools.odyssey_sim.vsim.pulses --scratch "$VSIM2"
.venv/bin/python -m tools.odyssey_sim.vsim.transfer \
  --scratch "$VSIM2" --baseline "$VSIM_SCRATCH/g6.traces.json"
```

`perturbation.py` implements dual plants and sine/triangular-ramp/finite-rise step
signals. All injected warmup frames receive the same time-dependent warp;
ONNX graph state retains the resulting image and feature histories. A literal
instantaneous step would demand infinite heading, so `step` uses a smooth rise.

`closed_loop.py` reads native applied-CAN commands and causal body/vehicle/tuning
caches, imports the existing real V0/NNLC controller adapter, and uses upstream
`clip_curvature` on each native control tick (nominal 100 Hz). Actual CAN intervals
are retained; gaps outside 5–25ms abort. Controller initialization occurs only in
scratch Params, including production extensions. Run **one controller per process**;
the scratch Params constructor binding is process-local. Factories have the existing
`factory(meta, Config) -> update(target_accel, measured_accel, state, future_plan)`
contract; commands are acceleration-positive. Controllers warm on recorded vehicle
state, then release at the requested window start. There is no pose reset after
release. Ineligible driver/blinker/activity samples abort the run; partial traces
and the measured reason are retained. Poses beyond 1m or .1rad abort rather than
pretend the road-plane approximation is trustworthy there.

`pulses.py` reads local maneuver schedules and native body yaw; it subtracts matched
zero-pulse holds from both observation and prediction, excludes driver overrides,
and reports sizes/directions separately. It never uses VSA LAT_ACCEL, and refuses
to substitute validation-route fitting for the absent 1a3 frozen-hold experiment.
The existing plant remains unchanged. `transfer.py` compares paired injected and
baseline perception, fits harmonic gain/phase, and reports a band-limited natural
training fit with its observational-identification limitations.

`metrics.closure_metrics` exports per-window near-lane RMS, period and spectral
power; those values alone cannot authorize ranking. See `report_vsim2.md` in scratch
for generation failures, missing coverage and exact next steps.

## Pass 3: discrimination and prospective validation

`campaign.py` discovers **all maximal** native autonomous stretches >=45s, with
v>5m/s, no driver override/blinker, finite context, and no >25ms CAN gap. It
requires both cameras throughout scoring plus 15s recurrent warmup. A segment is
ready only after both HEVC files exist and its next segment starts filling. The
inventory records ineligible/missing-video coverage rather than selecting a
convenient straight. Nothing reads the comma or writes to the log directory.

```sh
.venv/bin/python -m tools.odyssey_sim.vsim.campaign --scratch "$VSIM3" \
  --routes 000001a3 000001a7 \
  --arms stock v0 kp_half kp_double V1 S3 S5 friction bad_delay
# Held-out settings stay frozen; S3/S5 are prospective comparisons.
.venv/bin/python -m tools.odyssey_sim.vsim.campaign --scratch "$VSIM3" \
  --routes 000001aa 000001ab --arms stock S3 S5
.venv/bin/python -m tools.odyssey_sim.vsim.analyze --scratch "$VSIM3"
.venv/bin/python -m tools.odyssey_sim.vsim.bias --scratch "$VSIM3"
# Register the calculation before any new road drive/results; finalize after every named route is scanned.
.venv/bin/python -m tools.odyssey_sim.vsim.predictions --scratch "$VSIM3"
.venv/bin/python -m tools.odyssey_sim.vsim.predictions --scratch "$VSIM3" --finalize
```

`candidates.py` uses the pass-2 native V0 adapter. KP changes apply to the entire
V0 schedule. V1/S3/S5 use the same bounded evaluator and response-shaper metadata
as the production model A/B. `friction` adds the inverse FrictionAssist to stock
NNLC; `bad_delay` delays the complete stock command by 30 native control updates
(nominal 0.3s). Each run owns scratch Params and a separate process. Simulation
identity records the requested arm. Commands and body/relative-pose trajectories
are retained. Partial/domain-exit runs remain failures, never improvements.

`run.py` also prints `matched_curve_metrics` directly, using the same
`metrics.curve_scores` implementation as batch analysis. It also prints
`canonical_lane_metrics` from the shared `metrics.lane_scores` function; the
legacy unmasked closure diagnostics are labelled explicitly. Its per-event values
remain separate from the legacy nullable aggregate perception fields.

`analyze.py` reports paired-window mean differences and 95% percentile bootstrap
intervals (10,000 seeded window resamples; fewer than two windows has no interval).
These quantify conditional window uncertainty, **not** transport uncertainty for a
new road drive. Wander uses the existing 7–20s detrended FFT metric in cm RMS,
requiring both logged lane probabilities >0.5 for >80% of each window and
interpolating low-confidence holes. The logged mask is fixed across candidates;
insufficient visibility returns null. Unmasked diagnostics remain explicit.
Near-field cubic-fit diagnostics remain separately identified. Spectra and maximum
lane offset accompany pose departure. Curves are selected from the **logged**
plan using `curves.characterize.events` so candidates cannot change event selection.
Signed outside-directed width uses the middle 50% of each matched event,
requires >80% visible mid-curve support, and is null otherwise; body release
uses the 20% logged-plan-peak crossing. Body jerk is the RMS derivative of body
lateral acceleration resampled to 20Hz. `odyssey_wander --ab nnlc_model` currently
reports wander only; curve width (m) and jerk (m/s³) are additional metrics.

`bias.py` applies the natural regression to complete, visible training-route
free-running outputs and
compares its implied harmonic gain to the pass-2 measured injected response.
Differential fits also compare true relative pose with perceived lane regressors.
A known linear/noise calibration demonstrates attenuation algebra; it does not
establish the real model's noise level or identify the cause of the natural-fit
gap. Road geometry, feedback and omitted temporal dynamics remain confounders.
`prepare_extra.py` reuses extraction code for new local routes, without fitting,
and confines new caches to `$VSIM3/cache`. Use the canonical research scratch
`S/vsim3` for extra routes: `native_data` reuses the fixed existing `S` caches
first, then `S/vsim3/cache`, where `S` is defined by `calib.identify`.

Forecast registration preserves version history: the original unmasked formula
and the visibility-corrected formula are timestamped separately. Full-window
forecast inputs with unavailable lane metrics are excluded explicitly, not
replaced by zero; body motion remains scoreable without lane lines.

Only a completed discrimination test **and** held-out lane/curve closure can
license rankings. A small error against the log is not enough when pose departure
is small. Prospective road predictions must be timestamped before observing new
A/B-drive results; retain their input-route inventory and hashes.
