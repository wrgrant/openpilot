# Odyssey offline torque replay

Run from the repo with `PYTHONPATH=$PWD .venv/bin/python ...`.

```python
from tools.odyssey_sim.replay import ControllerConfig, replay_route, replay_configs
for frame in replay_route('000001aa', ControllerConfig(nnlc=True, delay=.45, version=0)):
  if frame['active'] and not frame['pressed']:
    torque = frame['command']
```

Route accepts a unique local route substring, a route directory, or an existing rlog file.
`replay_configs(route, [config, ...])` yields the same dictionaries with torque under
`config.name` and PID components under `config.name + '_p'/'_i'/'_f'/'_error'`.
`max_frames` bounds a smoke run. Frames contain monotonic seconds `t`, m/s `speed`,
`desired`/`actual` lateral acceleration in m/s², logged output and parameters.
`command` has logged `torqueState.output` / `actuators.torque` sign: opposite curvature.
This is controller output before Honda CAN rate limiting and EPS hardware response.

Default `version=0` deliberately executes sunnypilot's real legacy class, selected by
ControlsExt.initialize_lateral_control with EnforceTorqueControl=0. `version=1` executes
the real upstream-derived class requested in the brief. Both use the actual extension
and Odyssey JSON. No controller equations are reimplemented.

Inputs are sorted by logMonoTime within each segment. Snapshot incoming model, CS,
vehicle/live parameters and torque calibration on selfdriveState (controlsd's poll
message, matching process_replay.py's MessageBasedRcvCallback). Evaluate when the
paired carControl follows controlsState. Commands/curvature/activity and calibrated
pitch are taken from that recorded output. Replay all inactive/driver-override frames
to retain controller state, then filter for analysis. Safety freezing follows the
preceding logged command-versus-carOutput difference. Delay=None uses logged
lateralDelay for core and LagdToggle/LagdValueCache for extension; explicit delay
overrides both. NNLC history, PID, VM parameters and filters run continuously.

OPENPILOT_PREFIX is set to `simreplay` before constructing Params. Only this isolated
namespace is written. Do not initialize replay configurations concurrently in separate
threads/processes: their shared simreplay toggle initialization can race. Executing
several configurations within one replay_configs call is supported. The namespace is
left available for reruns. Device parameters and device/network access are unused.

The harness disables experimental jerk-aware, override, low-speed-angle, gain-cap,
NNLC A/B and deadband settings. These logs use v0, which never calls deadband
compensation even though its A/B parameter is enabled. Replaying v1 therefore means
stock v1 versus NNLC v1 without deadband; it does not reproduce these v0 logs.

Not a replacement for full process replay: the recorded desired curvature bypasses
planner clipping, and recorded calibrated orientation bypasses PoseCalibrator.
This isolates torque control and provides an importable sequential API for the
closed-loop worker; it does not simulate a changed vehicle trajectory. Full process
replay was inspected for triggering and Params/input conventions, not launched.

Local integration check: `.venv/bin/python tools/test_runner.py tools/odyssey_sim/replay/test_replay.py`. It requires the local 1aa rlogs. Ruff: `.venv/bin/ruff check tools/odyssey_sim/replay`.
