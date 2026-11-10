# Brief: driver-controlled A/B tests on the comma screen, a live data counter, and a drive map

Read `ODYSSEY.md` first: "What actually runs (code map)", "After every drive", "Working on the van (pitfalls)", and items 15, 19, 21, 23 and 25 of the carried patch. The A/B plumbing is `openpilot/sunnypilot/selfdrive/controls/lib/odyssey_ab.py`.

## Why

Road A/B tests on William's Odyssey switch arms by the clock (`ab_arm`: 60 s or 180 s blocks of logMonoTime). That fails him in three ways, in his words:

- "One test might run a nice straightaway for a while, but then there's a long sweeping curve. Maybe one of the arms got stuck on two long sweeping curves." The arms see different roads.
- "I know my streets around here. I know when there is going to be a minute or two of good solid straight or slightly curving road, so I could help by controlling that we give A about half the space, then switch to B."
- He cannot tell which arm is running, so he cannot say which one felt bad. On 9 November one arm shook the van badly and the scorer reported "too little data".

He asked for: "a UI that allows me to toggle between the different ones and gives me a cumulative count of how many minutes or seconds we've had of each, and what the target is. Even as it goes, if it can be counting up whether these are good data point miles, through an analysis algorithm in real time, or if they're not for some reason." And: "on a map, show different colours on the roads where different arms were active, which ones were valid, or which came out with too much noise."

The device is a comma 3X (the large `tizi` layout, raylib Python UI in `openpilot/selfdrive/ui/`, sunnypilot additions in `openpilot/selfdrive/ui/sunnypilot/`). William is the judge of how an arm feels; the software's job is to tell him what is running, let him choose, and count.

## What to build

### 1. Manual arm control (controls side)

- An **experiment** is a named list of arms. Keep the list in `odyssey_ab.py` as the single definition. Start with:
  - `lane`: off, on (`odyssey_lane.py`);
  - `bump`: off, kick, bump (`steer_deadband.py`, the existing `DEADBAND_ARMS`);
  - `nnlc_model`: stock, S3 (S5 is retired: it failed on the road);
  - `lowspeed`: base, firm (`low_speed_angle.py`).
- A persistent param selects the active experiment (empty = none) and whether it is manual. In manual mode the driver's chosen arm applies and the clock is ignored for that experiment; with manual off, today's clock rotation is unchanged. One experiment is active at a time in manual mode, and every other experiment's feature takes its always-on param value.
- The chosen arm reaches controlsd within half a second of a tap, without a blocking read in the 100 Hz loop. Each arm switch resets what the existing A/B switches reset (see `update_ab`, `friction_enabled`).
- **Safety and fallback.** Unknown experiment or arm names fall back to the first arm (always the "off" or stock arm). Everything stays behind params that default off. Nothing here changes steering when the experiment param is empty.

### 2. Good-data seconds, one definition

One pure function in `odyssey_ab.py` decides, per frame, whether the current moment is good data for the active experiment and why not otherwise. controlsd calls it live, and the offline scorer imports the same function. Start from:

- openpilot steering, no blinker and no lane change;
- speed inside the experiment's range (`lowspeed`: under 18 mph; the others: above 25 mph);
- at least 5 s since the last arm switch;
- no real driver override: only a wheel press lasting 1 s or more counts, because William's hands are off the wheel and shorter `steeringPressed` events are the sensor reading the wheel's own motion (`ODYSSEY.md`, measured facts);
- the model's inner lane lines visible (both probabilities above 0.5) for experiments scored on lane position.

Label each good second **straight** (desired lateral acceleration under 0.5 m/s²) or **curve**, and count the two separately per arm, so he can balance curves as well as time. Keep per-arm totals across drives in a persistent param (JSON, validated, reset from the UI). Defaults for the target: 300 straight seconds and 60 curve seconds per arm; make them constants beside the experiment list.

### 3. A record in the log

The scorer must be able to tell, from the rlog alone, which experiment and arm ran at every moment and whether the second counted. Emit a `cloudlog.event("odyssey_ab", ...)` (it lands in the rlog as a `logMessage`) on every change and once a second while steering: experiment, arm, manual, good, reason, straight or curve. If you find a cleaner existing carrier, use it and say why. Do not change capnp schemas.

### 4. The on-screen panel

On the onroad view, in the large layout:

- the experiment and the **current arm in large type**, readable at a glance;
- one row per arm: straight seconds and curve seconds against the targets, as numbers and a bar, with the arm that most needs data marked;
- a live indicator: counting, or the reason it is not ("under 25 mph", "blinker", "5 s after switch", "lane lines lost", "you're steering");
- **a tap switches to the next arm**, with touch targets large enough to hit while driving without looking closely; a long press or a second control resets the totals with a confirmation.
- The panel must not cover the road ahead, the speed, or alerts, and must not appear at all when no experiment is active.
- An offroad settings entry to pick the experiment, manual or clock, and reset totals. Follow the pattern of the existing sunnypilot settings pages.

William also reports that his steering torque bar is missing since the move to the current upstream. `TorqueBar` was off on the van and is now on; confirm from the code and the UI test harness that the bar (`openpilot/selfdrive/ui/mici/onroad/torque_bar.py`, drawn from `sunnypilot/onroad/hud_renderer.py` and `augmented_road_view.py`) renders in the large layout with steering active, and that his developer-panel elements (`sunnypilot/onroad/developer_ui/elements.py`, steering torque % and curve speed) still render with `DevUIInfo` 3. Fix what is broken.

### 5. Offline: the scorer and the map

- `tools/odyssey_wander.py` and `tools/odyssey_takeovers.py` read the arm from the log record when it is present and fall back to the clock for older drives. Their good-data selection uses the shared function.
- New `tools/odyssey_drive_map.py <route dir> [<route dir> ...] -o map.html`: a self-contained HTML map (Leaflet from cdnjs, OpenStreetMap tiles) of the drives with four colourings he can switch between: which arm was running; the sideways push he felt in newtons on an 80 kg person; the speed band; and which seconds counted and why the rest did not. A working first version is in `~/comma-logs/odyssey-evidence/drive_1109/` (`mapdata.py`, `map_template.html`, `gps.py`, `extract.py`): fold it into the tool, reading GPS from `gpsLocation`. Add a per-arm table under the map: minutes, straight and curve seconds counted, typical and worst push in newtons, direction changes per second.
- **Road-matched comparison.** Because he will now drive the same stretch on different arms, add to the scorer a comparison that pairs arms on the same piece of road (match by GPS position and heading, in bins of about 50 m) and reports the paired difference with an interval. Validate it as a null test on the existing drives, where the same roads were driven with no difference between "arms" (route list in `tools/odyssey_sim/reports/BRIEF_COMMON.md`; the 9 November routes are 1ac, 1ad, 1ae).

## End state (checks)

1. Unit tests cover: arm selection in manual and clock mode, the fallback on bad names, the good-data function's every reason, the totals param (validation, reset, persistence format), and the log record round trip into the scorer.
2. The real controlsd runs clean over a recorded minute with an experiment in manual mode on each arm: `tools/odyssey_sim/replay/controlsd_check.py` (extend it if needed so a chosen arm can be forced).
3. The panel is rendered by the repo's UI test harness (`openpilot/selfdrive/ui/tests/`) in at least these states, with screenshots saved under the scratch folder and shown in your report: no experiment (panel absent), counting, not counting with a reason, an arm at target, and the torque bar visible.
4. A simulated tap changes the arm, and the change reaches the controls-side selection in a test.
5. The 100 Hz controls loop gains no blocking I/O: measure the added time per frame and report it.
6. `tools/odyssey_drive_map.py` reproduces the 9 November map from the three routes, and the road-matched null test is reported.
7. `.venv/bin/python tools/test_runner.py` passes for every test file you touched or added, and `.venv/bin/ruff check` is clean on them.

## Rules

- Repo `~/Code/odyssey-pilot`, branch `odyssey-sim`. Never switch branches, never commit, never push; the lead reviews and commits. Never touch the comma or the van: the lead installs, and William tests the screen with the van on.
- Write in `openpilot/selfdrive/ui/`, `openpilot/sunnypilot/selfdrive/controls/lib/` (and its tests), `openpilot/selfdrive/controls/controlsd.py` only where a hook is needed, `openpilot/common/params_keys.h`, `tools/odyssey_wander.py`, `tools/odyssey_takeovers.py`, `tools/odyssey_drive_map.py`, `tools/tests/`, `tools/odyssey_sim/replay/controlsd_check.py`, and the scratch folder `scratchpad/abui/` under the session scratch directory named in `BRIEF_COMMON.md`. Another worker is writing `tools/odyssey_sim/shake/` at the same time: do not edit it. Do not edit `ODYSSEY.md`; put the text it needs in your report.
- A new params key needs the params library rebuilt before Python sees it: `PATH=$PWD/.venv/bin:$PATH .venv/bin/scons -j8 openpilot/common/libparams_c.dylib`.
- Python: `cd ~/Code/odyssey-pilot && PYTHONPATH=$PWD .venv/bin/python`. Tests are unittest through `tools/test_runner.py`; there is no pytest.
- Runtime code is numpy only, bounded, and falls back to stock behaviour on bad input.
- Match the surrounding code: small functions, short comments that say why, no new dependencies.

## Deliverable

`tools/odyssey_sim/reports/report_abui.md`: what William will see and do (with the screenshots), how to turn it on, the design decisions and why, each end-state check with its evidence, the text to add to `ODYSSEY.md`, and anything that could only be verified with the van on. Then finish with a short summary message.
