# Video simulator research: real-model perturbation loop

This is a **gated research prototype**, not a controller-ranking simulator.
It runs the repository's actual default driving ONNX, both recorded cameras,
causal calibration/delay/traffic/desire context and the graph's recurrent state.
It applies a road-plane homography before camera preprocessing, in one resample.
The model predicts the plan and action; no fitted planner replaces it.

The moving 1a3 replay reaches desired-curvature r=0.99983. Pass 2 adds dynamic,
kinematically consistent warps and a free-running real-controller loop around a
logged-trajectory perturbation plant. A training straight completes, but **held-out
closure and controller rankings remain unvalidated**: held-out video has not arrived.
The pulse gate is also incomplete: 1a3 contains an older suite without the frozen
no-pulse holds needed for the requested train/validation experiment.

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
These are unmasked perception diagnostics. Matched curve width, curve release and takeover metrics remain unavailable.
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
  tools/odyssey_sim/vsim/test_geometry.py tools/odyssey_sim/vsim/test_model.py
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

Use the same scratch dependency environment above; set a separate output directory:

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
