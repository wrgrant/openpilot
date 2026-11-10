# Brief: an automatic pulse test that measures what a small steering bump actually does

Read `ODYSSEY.md` first: "What actually runs (code map)", "Measured facts about this van" (the EPS bullets and the zero-crossing bullet), "After every drive", "Working on the van (pitfalls)", and items 11, 13, 26 and 27 of the carried patch. Then `report_abui.md` in this folder: the driver-controlled A/B panel, `LiveAB`, and the `odyssey_ab` log record are what you build on.

## Why

William's Odyssey ignores lane-keeping torque requests under about 3% of maximum, so the small corrections that hold a car in the middle of its lane never reach the wheel, and the van weaves. The fix he wants is a minimal bump: a short pulse just big enough to move the wheel, used only when a real offset has built up. Nobody has measured what such a pulse does on this van:

- how big and how long a pulse must be before the motor answers and the wheel moves at all;
- how far the wheel turns, how much the van moves, and how hard the driver is pushed sideways, for each pulse size and length;
- whether a train of pulses ("PWM") delivers the same as a small steady torque would;
- whether left and right differ, and whether a pulse that reverses the last movement behaves differently from one that continues it.

An earlier characterization drive (`openpilot/tools/lateral_maneuvers/odyssey_suite.py`) tried to add pulses on top of lane keeping and hold lane keeping's torque still from the 20 Hz maneuver daemon. Lane keeping still swung its torque by about ±0.1 during the holds and buried the pulses. That design is dead.

## The test William asked for

In his words: "It doesn't even run actual lane keeping. It only runs on a nice straight section, does the bumps, and pauses the test if it gets too far off centre, gets back on centre, then resumes, so it can still do as much data collection as it can. Each test lasts just a couple of seconds. I just know not to touch the wheel." During a shot lane centring is "totally disabled so it can't affect" the measurement.

So, with the test selected and the van engaged:

1. **Wait** for a good moment: steering active, hands off, no blinker or lane change, speed steady inside a band, the road ahead straight for the next few seconds (from the model's plan), inner lane lines visible, and the van near the lane centre and pointing along the lane.
2. **Shot** (about 2-3 s): the steering command is taken away from lane keeping and set by the test alone, at 100 Hz inside controlsd. A quiet lead-in at a known constant command, then the pulse or pulse train, then a quiet tail at the same constant command while the wheel and the van respond. Nothing else (the kick, the bump, friction assist, the lane correction) touches the command during a shot.
3. **Recover**: lane keeping steers normally until the van is centred, pointing along the lane and settled again, then the next shot. A shot ends early and lane keeping resumes at once if the van drifts too far from centre, a curve arrives, the driver takes the wheel, the blinker or brake comes on, or steering disengages. An aborted shot is recorded as aborted and repeated later.

It runs by itself for as long as the road allows, across as many drives as it takes, and remembers what it has finished.

## What to build

### 1. The shot controller (controls side)

- A small state machine (wait, lead-in, pulse, tail, recover) in its own module under `openpilot/sunnypilot/selfdrive/controls/lib/`, stepped from controlsd at 100 Hz, that replaces the final steering command during a shot. Hook it after `OdysseySteer.apply`, so the command that leaves controlsd during a shot is exactly the test's.
- The absolute command during a shot must be known and constant outside the pulse. Decide between a zero baseline and a frozen baseline (the average of what lane keeping was sending just before, which holds the van against road crown), justify the choice with the logs, and record the baseline with every shot. A baseline inside the dead zone and one outside it are different experiments.
- Lane keeping must come back smoothly: its controller keeps running while its output is discarded, so handle integrator wind-up, and bound the step at hand-over.
- **Limits.** Total commanded torque during a shot never exceeds 0.15 of maximum. Abort thresholds are constants at the top of the module with the reason for each: distance from lane centre, heading relative to the lane, upcoming curvature, speed band, driver torque (the pulse itself can trip `steeringPressed`, so use sustained driver torque, and say how you chose it), blinker, brake, lane-line confidence, and a hard time limit per shot.
- Selected as an experiment named `pulse` in the existing A/B selection (`OdysseyABExperiment`), with nothing changed when it is not selected. It has no arms: the panel shows its own status instead (see 3).
- Remove the dead design: the held-pulse path in `odyssey_suite.py`, the `torqueOffset` hook in controlsd, and anything of ours that only served it, unless something else still uses it. List what you removed and what you kept.

### 2. The shot plan

Start from this and improve it; say what you changed and why.

- **Single pulses:** heights 2, 3, 4, 5, 6, 8, 10 and 12% of maximum; lengths 0.05, 0.1, 0.2 and 0.4 s; both directions. The carcontroller's rate limit (`STEER_DELTA_UP`) shapes the edges of short pulses: account for it and log what was actually sent (`carOutput`).
- **Pulse trains:** heights 5, 6 and 8%; on for 20, 40 and 60% of each period; periods 0.25 and 0.5 s; 1.5 s long; both directions. These answer whether pulse-width modulation delivers a small average torque.
- **Shams:** shots with no pulse, about one in ten, to measure the noise floor.
- **Order:** randomized, with one full pass over every condition before any repeat, so a partial test is still a balanced one. Three repeats of each. Two speed bands, 25-45 mph and above 45 mph, tracked separately. Record for each shot which way the wheel last moved before it.
- Persist progress in a param so the test resumes on the next drive, and can be reset from the settings page.

### 3. On the screen

With `pulse` selected, the panel (`openpilot/selfdrive/ui/sunnypilot/onroad/odyssey_ab_panel.py`) shows, in the same large type: the state in plain words ("waiting for a straight", "pulsing", "re-centring", "paused: you're steering", "done"), shots finished out of the total for the current speed band, and the last shot's result in one line (for example "5% for 0.2 s: wheel moved 0.3°"). A tap pauses and resumes the test. Add `pulse` to the experiment choices on the settings page.

### 4. The record and the report

- Log every shot with `cloudlog.event("odyssey_pulse", ...)`: an id, the condition, the baseline, start and end times on the monotonic clock, speed, and whether it completed or why it aborted.
- A report tool (extend `tools/odyssey_eps_report.py` or replace it) that reads the route folders and gives, per pulse height and length and per train setting, against the shams: the motor's answer (`STEER_MOTOR_TORQUE` on CAN), how far and how fast the wheel turned (use the rate-integrated angle the existing report computes, which is finer than CAN's 0.1° steps), the van's sideways acceleration (speed × yaw rate), and the push on the driver in newtons (William weighs 75 kg) with direction changes per second. Then:
  - the smallest pulse that reliably moves the wheel, with an interval, per speed band and direction;
  - the average torque a train delivers against its nominal average, and so the corrected on-time for `steer_deadband.bump_comp`;
  - which pulses move the wheel while pushing the driver less than his normal ±7-8 N.
  - a JSON summary of the same, for the simulator work in `tools/odyssey_sim/shake/` to fit against.
- Say how many completed shots the report needs before each conclusion is trustworthy, so the panel's total means something.

## End state (checks)

1. Unit tests for the state machine: every transition and every abort reason, the command during each phase, the limits, the hand-over back to lane keeping, the plan's ordering and persistence.
2. `tools/odyssey_sim/replay/controlsd_check.py` runs the real controlsd over a recorded straight highway minute with `pulse` selected: controlsd stays up, the command during shots equals the plan exactly (compare against the logged plan), and with `pulse` not selected the output is identical to before your change.
3. A closed-loop check that the van stays in its lane: simulate shots with the existing plant (`tools/odyssey_sim/plant/` or the pass-1 physical model in `tools/odyssey_sim/shake/`) at the worst allowed baseline and pulse, and report the largest drift from centre before the abort or the tail ends. State the plants' known weaknesses beside the number.
4. The added time per 100 Hz frame, measured, with no blocking I/O in the loop.
5. The panel rendered by the UI test harness in each state, screenshots in your report.
6. The report tool run on synthetic logs with a known answer (a wheel that moves only above a set threshold) recovers that threshold.
7. Tests pass through `.venv/bin/python tools/test_runner.py` for every file you touched or added (run the UI render tests in their own call), and `.venv/bin/ruff check` is clean.

## Rules

- Repo `~/Code/odyssey-pilot`, branch `odyssey-sim`. Never switch branches, never commit, never push; the lead reviews and commits. Never touch the comma or the van: the lead installs, and William runs the test.
- Write in `openpilot/sunnypilot/selfdrive/controls/lib/` (and its tests), `openpilot/selfdrive/controls/controlsd.py` only for the hook, `openpilot/selfdrive/ui/sunnypilot/`, `openpilot/selfdrive/ui/tests/`, `openpilot/tools/lateral_maneuvers/` (to remove the dead design), `openpilot/common/params_keys.h`, `tools/odyssey_eps_report.py` or its replacement, `tools/tests/`, `tools/odyssey_sim/replay/controlsd_check.py`, and the scratch folder `scratchpad/pulse/` under `/private/tmp/claude-501/-Users-williamgrant-Documents-My-vault/ae63c39a-5411-41d2-abc4-99cb7e2003e6/scratchpad/`. Another worker is writing `tools/odyssey_sim/shake/` at the same time: read it, do not edit it. Do not edit `ODYSSEY.md`; put the text it needs in your report.
- A new params key needs the params library rebuilt before Python sees it: `PATH=$PWD/.venv/bin:$PATH .venv/bin/scons -j8 openpilot/common/libparams_c.dylib`.
- Python: `cd ~/Code/odyssey-pilot && PYTHONPATH=$PWD .venv/bin/python`. Tests are unittest through `tools/test_runner.py`; there is no pytest.
- Runtime code is numpy only, bounded, and falls back to normal lane keeping on any bad input or internal fault: a fault in the test must never stop steering.
- This test takes steering away from lane keeping on a public road with the driver's hands off. Be conservative, and write down every assumption about what keeps it safe.

## Deliverable

`tools/odyssey_sim/reports/report_pulse.md`: what William will see and do, how to turn it on, the shot plan with the time it needs, each limit and why, each end-state check with its evidence, what you removed, the text to add to `ODYSSEY.md`, and what can only be verified with the van on. Then a short final summary message listing each end-state check as met or not met.
