# Brief: make the simulator reproduce the 9 November shake, from physics

Read `BRIEF_COMMON.md` in this folder first (goals, data paths, rules), then `ODYSSEY.md`: "Measured facts about this van", "After every drive", and items 19-25 of the carried patch. Two corrections to the shared brief: William is awake and reads reports, and **body motion he feels outranks everything except the path** (the shared brief's "jitter does not matter" applies to the wheel rim only).

## What happened

On 9 November the van drove three routes (`000001ac--114fc8ccff`, `000001ad--fca502745a`, `000001ae--e34de72a0e`) rotating three NNLC models every 180 s by the clock (block k = floor(logMonoTime_s / 180); model = stock, S3, S5 for k % 3 = 0, 1, 2; lane correction on when k is odd).

- **S5 on US-169 north at 60 mph** (route 1ac, block 2, 5.7 to 8.7 minutes in) shook the van side to side: the push on an 80 kg person was ±20 N typical and ±40-50 N at worst, 1.3 times a second, 2.7 direction changes a second, 1.1 m/s³ rms of lateral jerk. William: "comically bad, almost nauseous". Earlier drives with stock NNLC are ±7-9 N at about 0.3 Hz.
- **The video simulator's locked forecast** (`vsim3/predictions.json`) said S5 would add +0.04 m/s³ of body jerk over stock. The road said +0.34. No simulator in this repo predicted the shake.
- **The mechanism William named:** the van-trained models act like an audio compressor around zero. A tenth of a second after the command crosses zero its size is a median 0.03 of max with stock, 0.08-0.12 with S3 and 0.17-0.19 with S5, against a motor deadband of about 0.03, and the command crosses zero 3.8-4.3 times a second with S5. Every crossing became a push the van answered. S5 also carries `error_gain` 2.0 in its `odyssey_smoothing` block (`nnlc/odyssey_models/`, `NnlcResponseShaper` in `nnlc/nnlc.py`).
- S3 was calm with the lane correction off and shook with the unsmoothed lane correction on. The lane correction has since been low-passed at 0.5 s.

This is the richest excitation these logs have ever had: three controllers of very different gain on the same roads, with command swings of 0.04 to 0.3 of max at 0.3 to 3 Hz. William's request: use it to characterize the van from physics, so that a simulator that matches it can be trusted to tune steering to "ridiculously smooth".

## Data

- 100 Hz tables for 1a7, 1aa, 1ab, 1ac, 1ad, 1ae in `~/comma-logs/odyssey-evidence/drive_1109/*.npz`: `t` (logMonoTime s), `seg`, `v`, `lat` (steering active), `pressed`, `drv` (driver torque sensor), `blinker`, `angle`, `rate` (wheel deg, deg/s), `torque` (actuators.torque), `des_curv`, `cur_curv`, `des_la`, `act_la`, `l_y`, `r_y`, `l_p`, `r_p` (inner lane lines at x=0 and their probabilities), `yaw` (carControl.angularVelocity z). The scripts beside them (`extract.py`, `felt.py`, `shake.py`, `weave.py`) made the numbers above.
- The rlogs hold more: `accelerometer` and `gyroscope` at 100 Hz (roll rate, measured lateral acceleration at the windshield), CAN (`STEER_MOTOR_TORQUE`, `STEER_TORQUE_SENSOR`; see how `~/comma-logs/odyssey-evidence/plant_data/*.npz` was extracted for earlier routes), `modelV2`, and road and wide camera video for all six routes.
- William says his hands were off the wheel except in curves where the van ran wide, so a `steeringPressed` flag on a straight is the sensor reading the wheel's own motion, not a hand.
- The van: 2019 Honda Odyssey Elite. Use published curb weight, wheelbase, track, steering ratio and tyre size, and cite them. William can give the occupants and where they sat on each drive; list what you need from him in the report rather than guessing silently.

## Tasks

### 1. A physical model of command to felt motion

Build the model from first principles and fit only what physics leaves free. At least:

- EPS: the command-to-motor-torque map (deadband, the cap at about 75 raw units, lag), using measured motor torque.
- Steering column and rack: inertia, a friction model that reproduces stick-slip (Coulomb plus viscous at minimum; try LuGre or Dahl if the simple one fails), self-aligning torque that grows with speed.
- Lateral dynamics: the single-track ("bicycle") model with front and rear cornering stiffness and yaw inertia, with tyre relaxation length if the phase needs it.
- Roll: one roll degree of freedom (roll stiffness, damping, roll centre and sprung-mass height), because the comma sits high on the windshield and a minivan's roll mode is near the 1-2 Hz of the shake. Say what the driver's head and seat feel as distinct from what the device measures.

For each parameter give the value, how it was identified (published, fitted, or assumed), and its uncertainty. Say plainly which parameters this data cannot pin down. Cite the vehicle-dynamics literature you rely on.

### 2. Gates: does it match the road?

Fit on route 1ac and earlier routes. Hold out 1ad and 1ae completely.

- **Gate 1, open loop.** Drive the model with the logged command on held-out blocks. In the 0.3-3 Hz band, the wheel angle and the yaw-rate lateral acceleration must match the log: frequency-response gain within ±20% and phase within ±20° at 0.5, 1.0, 1.5 and 2.0 Hz, per speed band (25-45 mph, above 45 mph), and band-passed time-domain correlation of at least 0.8. Report the same for stock, S3 and S5 blocks separately: one plant must explain all three.
- **Gate 2, closed loop.** Run the real controllers (`LatControlTorqueV0` plus NNLC with each model JSON and its response shaper, plus `OdysseySteer`; the validated harness is `tools/odyssey_sim/replay/`) against the model, with the logged desired curvature as the plan. For stock, S3 and S5 the simulation must reproduce, within ±25% of the road value on held-out blocks: the command's zero crossings per second, its median size 0.1 s after a crossing, and the 0.3-3 Hz rms of command, wheel angle and body lateral acceleration. S5's oscillation peak must land within 0.3 Hz of the road's. The ranking stock < S3 (lane correction off) < S5 must come out.
- **Explain the miss.** State which missing element made the earlier simulators under-predict S5 by ten times, with an ablation that shows it (remove the element, the shake disappears).

A failed gate is a result. Report exactly where the model departs from the road and what data would close the gap. William is willing to drive a purpose-built excitation run (a chirp or a pulse ladder he arms himself on a straight road); if you need one, specify it: amplitudes, frequencies, duration, speed, and what each part identifies.

### 3. Only if gates 1 and 2 pass: design the quiet-deadband bump

William's design, in his words: "we basically ran a compressor: every time it crosses zero we pushed it to a measurable point either side. Maybe the middle dead zone can actually be a dead zone, and as you approach a real, low-pass-filtered offset the car is trying to correct (not in the noise), that is where you give a bump: just the minimal bump that actually drives the wheel to point the right way."

- Formalize it: no output while the smoothed request sits inside a band; when a filtered, sustained error emerges (a Schmitt trigger, so noise cannot chatter it), one minimal pulse just above breakaway in that direction, then quiet again. Compare it with the existing kick and bump (`steer_deadband.py`) and friction assist (`odyssey_wander.py`).
- In the validated simulator, map the trade: the 7-20 s lane weave against the push on an 80 kg person (typical peak in newtons, and direction changes per second). Find the settings that cut the weave while the push stays at or under the earlier drives' ±7-9 N and well under one direction change a second from the controller.
- Deliver candidates as a standalone numpy module with tests, behind a default-off param, with integration notes. Do not wire it into runtime code. Lock a forecast for each candidate (weave, push, direction changes) in a JSON file before any road test.

### 4. The perception ladder

William offered to run an experiment to find what he can and cannot feel: the comma injects a small steering oscillation or pulse train on a straight road when he arms it, shows the measured sideways force, and he reports whether he felt it.

- Find the published thresholds for whole-body lateral acceleration and jerk between 0.3 and 3 Hz (vestibular detection thresholds, ISO 2631-1 comfort weighting, automated-driving passenger-comfort studies) and cite them. State what they predict for an 80 kg seated driver in newtons.
- Specify the ladder: frequencies, force levels (from clearly imperceptible to the S5 level), order, repeats, duration of each step, the speed and road it needs, and how his yes/no answers turn into a threshold with an uncertainty. Use the fitted model to turn each force level into a command amplitude. The existing maneuver framework is `openpilot/tools/lateral_maneuvers/` (`odyssey_suite.py`); say what it needs to run the ladder. Specification only: another worker owns the on-device code.

## Rules

- Repo `~/Code/odyssey-pilot`, branch `odyssey-sim`. Never switch branches, never commit, never push, never touch the comma or the van.
- Write only in `tools/odyssey_sim/shake/` (new) and the scratch folder `scratchpad/shake/` under the session scratch directory named in `BRIEF_COMMON.md`. Another worker is editing `openpilot/selfdrive/ui/`, `openpilot/sunnypilot/selfdrive/controls/`, `tools/odyssey_wander.py`, `tools/odyssey_takeovers.py` and `tools/odyssey_drive_map.py` at the same time: do not edit those.
- Tests with `.venv/bin/python tools/test_runner.py <paths>`, lint with `.venv/bin/ruff check`. Large caches stay out of git (scratch or `~/comma-logs/odyssey-evidence/`).
- Report every force in newtons on an 80 kg person next to the m/s², and every oscillation as times per second.

## Deliverable

`tools/odyssey_sim/reports/report_shake.md`:

- a plain-language summary for William first: does the simulator now reproduce what he felt on US-169, yes or no, with the numbers side by side;
- the model and its parameter table;
- each gate's result with plots saved beside the report;
- the explanation of the earlier miss;
- the bump design and locked forecasts, or the reason they are not yet justified;
- the perception ladder specification with sources;
- what you need from William (occupants, an excitation drive).

Then finish with a short summary message.
