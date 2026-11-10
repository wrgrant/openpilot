# Brief: a drive companion that reads the pulse test's shots minute by minute and says what is still needed

Read `ODYSSEY.md` first: "After every drive", "Working on the van (pitfalls)", "Measured facts about this van" (the EPS bullets), and items 13, 26, 27 and 28 of the carried patch. Then `openpilot/sunnypilot/selfdrive/controls/lib/odyssey_pulse.py` (the pulse test: `session()`, `Condition`, the records it logs) and `tools/odyssey_eps_report.py` (the current report, written for an older shot plan).

## Why

William has 10-15 minutes of straight highway per trip and 20 minutes of driving to reach it and get home again, so a trip must not come back with unusable data. He asked for an agent in the loop during the drive: the comma on his phone's hotspot, its logs pulled as they are written, analysed at once, and a short answer on the comma's screen: is the data good, what is still missing, should another shot be run. The lead agent (on the Mac Studio, reaching the comma over ssh) runs this tool during the drive and is woken by it at decision points.

What the pulse test does: on a straight, it takes the steering command away from lane keeping for 1.3-2.2 s and sends one signal, then hands back. One pass is 26 shots (`session()`):

- `square`: +level then -level for 0.1 s each, levels 2, 4, 6, 8, 10, 12% of maximum: the EPS map (motor torque at each command level);
- `ramp`: 2, 3, 4, 5, 6, 8% for 0.15 s each, one direction: the level at which the wheel starts to move;
- `pair`: a pulse of 4, 6 or 8% for 0.05, 0.1 or 0.2 s, 0.5 s of nothing, then the opposite pulse: the bump's own pulse and its reversal;
- `train`: 6 or 8% on for 30 or 60% of each 0.25 s, for 1 s: does pulse-width modulation deliver a small average torque;
- `chirp`: a sine sweeping 0.5 to 3 Hz at 5 or 9% for 1.5 s: the frequency response a simulator has to match;
- `sham`: nothing, for 1 s: the noise floor.

What is in the log for each shot (rlog `logMessage` entries from `cloudlog.event("odyssey_pulse", ...)`, read them with `odyssey_eps_report.pulse_record`): a start record (`phase: start`, `id`, `condition`, `baseline` 0, `released`, `start` on the message clock, `speed`, `band`, `angle_start`, `last_direction`), a frame record every 10 ms of the shot (`phase: frame`, `id`, `mono_s`, `command`, `carOutput`, `state`), and an end record (`end`, `completed`, `abort` reason, `wheel_delta`). The rlog also holds, at 100 Hz, `carControl` (the command sent), `carOutput`, `carState` (wheel angle in 0.1° steps, wheel rate, driver torque sensor, speed), CAN (`STEER_MOTOR_TORQUE`, unsigned), the raw `gyroscope`, and `modelV2`. The pose-filter yaw (`carControl.angularVelocity`) is 70-80 ms later than the raw gyro; say which you use.

On the comma, live: `/dev/shm/params/d/OdysseyABStatus` is a JSON status rewritten ten times a second (`state`, `reason`, `finished`, `total`, `band`, `last_result`, `paused`, `wheel`, `aborts`, and `queue` for queued shots left). The lead agent is adding two params the companion writes: `/dev/shm/params/d/OdysseyAgentNote` (text shown on the panel, two lines of about 34 characters) and `/data/params/d/OdysseyPulseQueue` (a JSON list of extra shots, each `{"kind", "height", "duration", "direction", "period", "duty"}` as in `Condition`, run before the remaining plan; write it atomically, to a temporary file then rename). Closed log segments are `/data/media/0/realdata/<route>--<n>/rlog.zst`; a segment still being written has an `rlog.lock` beside it.

## What to build

### 1. Shot analysis (rework `tools/odyssey_eps_report.py`; one tool, not two)

For every completed shot, from the logs alone:

- **square:** motor torque at each command level and sign (settled part of each half-step), so the map of command to motor torque, its dead zone and its slope come out with intervals; and whether left and right differ.
- **ramp:** the wheel's movement by the end of each level, using the rate-integrated angle (`fine_angle`, finer than the 0.1° steps), against the shams' noise: the first level that moves the wheel, as a bracket.
- **pair:** for the first pulse (from rest) and the second (a reversal) separately: how far the wheel moved, its peak rate, the motor's peak, and the delay from command to motor and to wheel.
- **train:** the wheel's movement and mean motor torque over the train against a steady command of the same average (take the steady reference from the square and ramp shots), so the report can say what on-time delivers what.
- **chirp:** gain and phase from command to wheel rate and from wheel angle to yaw rate at the frequencies the sweep covers, with coherence; these are for the simulator in `tools/odyssey_sim/shake/` (give them in a JSON that states signal sources and delays).
- **every shot:** the push on the driver in newtons (William weighs 75 kg: 75 x the peak change in speed x yaw rate, 0.3-3 Hz) and how many times a second it reversed; the largest wheel-sensor reading; how far the van moved in its lane; and whether the shot was clean (completed, sensor under its limit, enough samples).
- **what is still missing:** against `session()`, which shots have no clean result yet, and which conclusions are not yet supported (for example "the breakaway level is bracketed only between 4% and 8%").

Output: a JSON summary, a plain-language text report, and `note(summary)`: at most two lines of 34 characters for the panel, saying the most useful thing right now (for example `14 of 26 good. Wheel moves at 5%.` / `Need: 2 sweeps, ramp left.`).

Also `suggest(summary, limit=6)`: extra shots worth queuing, as `Condition` dicts inside the test's own limits (heights up to 12%, a shot shorter than `HARD_SHOT_S`): halve the bracket on the breakaway level, repeat a shot whose result disagrees with its mirror image, add a pulse length between two that straddle "moves" and "does not move". It must be deterministic and explain each suggestion in one line.

### 2. The companion (`tools/odyssey_drive_companion.py`)

`odyssey_drive_companion.py --host comma --out ~/comma-logs [--queue] [--max-minutes N]`

- Finds the drive in progress on the device, pulls each closed segment's `rlog.zst` as it appears (rsync over ssh, short timeouts, retries; the link is a phone hotspot or LTE and will drop), and reads the live status every few seconds.
- After each new segment: runs the shot analysis over the drive so far, writes the note to the device, and prints one line (time, segment, shots clean of total, the note). With `--queue` it also writes `suggest()`'s shots to the queue param, at most six outstanding at a time. It may set `OdysseyPulsePaused` to 1 if it sees a fault pattern (the same stop reason five times running, or shots completing with no samples). It never unpauses, never changes the test or arm, never touches any other param or file on the device, and never restarts anything.
- Exits, printing a summary that says why, when: a pass is complete; a fault pattern appears; nothing new has arrived for three minutes (the drive ended or the link is gone); or `--max-minutes` is reached. The lead agent is woken by the exit and decides what next, so the summary must stand alone: what is done, what is missing, what it recommends.
- All device access goes through one small class with a local stand-in, so the whole loop can be tested without a device: the stand-in serves a recorded route from a folder, releasing one segment at a time.

## End state (checks)

1. Synthetic-log tests with known answers for each shot kind: a motor with a set dead zone and slope (the square shot recovers both), a wheel that moves only above a set level (the ramp brackets it), a reversal that moves less than a pulse from rest (the pair shows it), a train that under-delivers (the report says by how much), a known second-order response (the chirp recovers gain and phase within 15% and 15°).
2. `note()` and `suggest()` tests: length limits, determinism, and that suggestions stay inside the limits and stop when nothing is uncertain.
3. The companion run end to end against the stand-in device with a synthetic drive that completes a pass: it pulls every segment, the notes progress, and it exits with "pass complete"; and a second drive with a fault pattern exits with that. Include a dropped connection in the middle.
4. Run the analysis on the 9 November drives (`~/comma-logs/04b44e4c4fe44232_000001ac--114fc8ccff`, `...ad--fca502745a`, `...ae--e34de72a0e`), which contain no shots: it must say so cleanly, not fail.
5. `.venv/bin/python tools/test_runner.py` passes for every test file you touched or added, and `.venv/bin/ruff check` is clean.

## Rules

- Repo `~/Code/odyssey-pilot`, branch `odyssey-sim`. Never switch branches, never commit, never push; the lead reviews and commits. **Never connect to the comma or the van**: no ssh, no rsync to a real host. Test against the stand-in only.
- Write in `tools/odyssey_eps_report.py`, `tools/odyssey_drive_companion.py`, `tools/tests/`, and the scratch folder `scratchpad/companion/` under `/private/tmp/claude-501/-Users-williamgrant-Documents-My-vault/ae63c39a-5411-41d2-abc4-99cb7e2003e6/scratchpad/`. The lead is editing `openpilot/` at the same time: do not edit it. Do not edit `ODYSSEY.md`; put the text it needs in your report.
- Python: `cd ~/Code/odyssey-pilot && PYTHONPATH=$PWD .venv/bin/python`; numpy only (no scipy) in anything the companion runs. Tests are unittest through `tools/test_runner.py`.
- Every force in newtons on a 75 kg person, every oscillation as times per second, every command as a percentage of maximum.

## Deliverable

`tools/odyssey_sim/reports/report_companion.md`: what the lead runs and what it prints, what William sees on the panel, each analysis with its method and its limits, each end-state check with its evidence, and the text to add to `ODYSSEY.md`. Then a short final summary message listing each end-state check as met or not met.
