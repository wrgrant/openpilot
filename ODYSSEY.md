# Odyssey branch: sunnypilot + factory longitudinal on Honda Nidec

This branch, `odyssey-stock-long` in `wrgrant/openpilot`, is stock sunnypilot plus one carried patch. With the patch, Honda Nidec cars (William's 2018–2020 Odyssey) can keep the factory ACC for gas and brake while sunnypilot only steers. NNLC and the driving-model selector come from sunnypilot unchanged.

## Base

- sunnypilot `staging` cut `staging/2026.003.000/2026.11.04-5003`, master commit `932ff06d9` (2026-11-04).
- This base reads model catalog `driving_models_v25.json`, which includes TSFDOM. The stable `release-tizi` (v2026.002.002) reads v17, which does not.

## The carried patch

Two repos, five commits in total.

**`wrgrant/opendbc`, branch `odyssey-stock-long`** (on top of the sunnypilot/opendbc pin `aa9bff6f`):

1. *Honda Nidec: stock longitudinal safety mode.* `opendbc/safety/modes/honda.h` gets a new SP safety bit, `HondaSafetyFlagsSP.STOCK_LONGITUDINAL = 4`. With it set:
   - The TX allow-list is steering (0xE4/0x194), LKAS_HUD (0x33D), and SCM_BUTTONS (0x1A6/0x296) on bus 0. BRAKE_COMMAND (0x1FA) and ACC_HUD (0x30C, which carries PCM gas and speed) are not allowed.
   - The stock ACC module's own BRAKE_COMMAND and ACC_HUD are always forwarded.
   - SCM_BUTTONS is cancel-only while controls are not allowed.
   - If the gas interceptor bit is also set, the interceptor wins.
   - Tests are in `opendbc/safety/tests/test_honda.py` (`TestHondaNidecStockLong*`).
2. *Honda Nidec: HondaEnforceStockLongitudinal param.* `opendbc/sunnypilot/car/interfaces.py::_initialize_honda` reads the param. On a Nidec with no interceptor it sets `HondaFlagsSP.STOCK_LONGITUDINAL = 16` and the safety bit, and sets `openpilotLongitudinalControl=False` and `pcmCruise=True`. The carcontroller then sends no brake or ACC_HUD, and it cancels the stock ACC through SCM_BUTTONS. BOSCH_SUPPLEMENTAL_1 is now sent only on Bosch. Tests are in `opendbc/sunnypilot/car/honda/test_honda.py`.

**`wrgrant/openpilot`, branch `odyssey-stock-long`:**

3. The opendbc submodule points at `wrgrant/opendbc` `odyssey-stock-long`.
4. The `HondaEnforceStockLongitudinal` param key goes in `openpilot/common/params_keys.h` and the car-interface param list (`openpilot/sunnypilot/selfdrive/car/interfaces.py`). There is also a Vehicle → Honda toggle, "Nidec: Enforce Factory Longitudinal Control" (`openpilot/selfdrive/ui/sunnypilot/layouts/settings/vehicle/brands/honda.py`). It can only be changed offroad, it has a confirm dialog, it is disabled on Bosch, and it uses `gui_app.push_widget`.
5. This file.

**`wrgrant/openpilot`, branch `odyssey-ui`** (on top of `odyssey-stock-long`), UI additions:

6. *Lead chevron with factory longitudinal.* `openpilot/selfdrive/ui/onroad/model_renderer.py` draws the lead chevron and Chevron Info (distance, speed, time-to-collision) whenever `radarState` is valid. Upstream only draws them when openpilot does longitudinal.
7. *Side panel and Steering Arc.* The right-hand developer column (`DevUIInfo`) shows CURVE SPD, STEER TRQ (commanded torque as a % of the car's max, the same signal the Steering Arc draws; blank while the driver is steering), steering angle, and desired/actual lateral accel. Each row puts the label and its unit upright on one line, and the value is right-aligned with fixed-width digits so it doesn't jitter. Lead distance and speed moved to the chevron. The Steering Arc (`TorqueBar`) is on by default, keeps one thickness at every torque, and has no centre dot. It reads like a VU meter: square-seamed white (0–60% of the end stop), yellow (60–85%) and red (85–100%) zones, with thin square seams between zones, rounded ends only at the two outer stops, the unlit zones shown faintly so you can see where each starts, and the fill grows through each zone in that zone's colour (`meter=True`); the fill keeps the rounded shape of the outer ends, and a red outline goes around the whole bar while commanded torque is pinned at 100%, the state that leads to the take-over alert. Code: `openpilot/selfdrive/ui/sunnypilot/onroad/developer_ui/`, `openpilot/selfdrive/ui/mici/onroad/torque_bar.py` (`fixed_height`, `center_dot`). Tests: `developer_ui/tests/test_odyssey_elements.py`.
8. *UI replay.* `tools/odyssey_ui_replay.py` plays a recorded drive (rlog + qcamera) through the real onroad UI on a Mac and records an MP4, so UI changes can be checked without the device. Usage is in the file's docstring.
9. *Green light chime delay.* `GreenLightAlertDelay` (Visuals → Green Light Chime Delay: Off, 0.5 s, 1 s, 1.5 s; default 0.5 s) holds the Green Traffic Light Alert chime for that long after the light turns green, and skips it if the driver lifts off the brake, presses the gas or moves in that time. With Brake Hold (foot already off the pedal) it chimes after the delay unless the van moves. Code: `sunnypilot/selfdrive/controls/lib/e2e_alerts_helper.py`. Tests: `sunnypilot/selfdrive/controls/lib/tests/test_green_light_delay.py`.
10. *Wander meter.* `tools/odyssey_wander.py <route_dir>` finds stretches where openpilot steers alone and measures the slow 7-20 s lane ping-pong: lane-position swing, its period, how much the requested lateral accel swings, and how closely the actual follows. A low follow ratio means the swing is in what openpilot asks for (model/planner, camera offset, steer delay), not the controller or the steering. Needs daytime highway drives for the lane-position column.
11. *Steering deadband kick.* `SteerDeadbandComp` (normalized torque, default 0 = off) adds that much torque in the direction of any steering request, blended in over 0.01 around zero. The Odyssey's EPS motor gives essentially nothing below about 150 of 4096 (3.7%), and openpilot's friction term only fires on an error the 0.1° angle sensor can see, so the small corrections that hold the lane never happen. Code: `sunnypilot/selfdrive/controls/lib/steer_deadband.py`, applied after NNLC in `latcontrol_torque.py`. Set on the device with `echo -n 0.03 > /data/params/d/SteerDeadbandComp` (it re-reads every 5 s).
12. *Lateral simulator.* `tools/odyssey_lateral_sim.py --road <route dirs>` drives the van in closed loop on the curves recorded on real highway drives. It models openpilot's request as road + 0.34·offset + 0.91·sideways speed (fitted from routes 18c and 19c, R² 0.98), the torque controller, and the steering: motor deadband, friction, lag, a one-sided pull, and the 0.1° sensor. It prints the same numbers as the wander meter, compares ideal steering, this van, and deadband-kick settings, and its steering values come from the characterization drive (item 13). Normal driving can't pin those values down: friction 0.02-0.08 and lag 0.25-0.6 s all fit routes 18c and 19c. Under every fit, a 0.03-0.037 kick cuts the 7-20 s wander from about 7.9 to 6.1-6.4 cm (with perfect steering it would be about 4.1 cm), and above 0.05 the torque starts to buzz.
13. *Steering characterization drive.* `openpilot/tools/lateral_maneuvers/odyssey_suite.py` runs small push-pull-push lateral-accel requests (0.04-0.1 m/s², at most about 25 cm sideways, back on line at the end) at 50 and 65 mph, left-first and right-first, 3 runs each, about 7 minutes of maneuvering. Setup while parked: Settings > Developer > Lateral Maneuver Mode (it clears itself when you turn the van off). This suite is the default on this branch (`LateralManeuverSuite` = `odyssey`). On a straight, flat, empty highway, set ACC to the speed shown; each run starts after a moment of steady straight driving and aborts if you touch the wheel or gas. Keep your hands just off the wheel so your torque doesn't mix in. Afterwards: `tools/odyssey_eps_report.py <route dir> --plot eps.png` gives delays, breakaway torque left/right and away/toward straight, the friction band, and values to plug into the simulator.

The param key matches mvl-boston's fork, so a device that already has it set carries the setting over.

## Rebasing onto a new sunnypilot release

```bash
cd ~/Code/odyssey-pilot
git fetch origin                       # sunnypilot/sunnypilot
NEW=<new sunnypilot commit or tag>     # e.g. the next staging/... tag or release-tizi's master commit
git checkout -b odyssey-stock-long-<version> $NEW
git submodule update --init opendbc_repo

# opendbc: rebase the two patch commits onto the new pin
cd opendbc_repo
git fetch wrgrant odyssey-stock-long
git checkout -b odyssey-stock-long-<version>
git cherry-pick dc1156da 3acb7c30      # or the current tips of wrgrant/opendbc odyssey-stock-long
source setup.sh && (cd opendbc/safety/tests && bash test.sh) && python -m unittest opendbc.sunnypilot.car.honda.test_honda
git push wrgrant HEAD
cd ..

# openpilot: point the submodule at the fork and re-apply the param/UI commit
git config -f .gitmodules submodule.opendbc.url https://github.com/wrgrant/opendbc.git
git add .gitmodules opendbc_repo
git commit -m "opendbc: wrgrant/opendbc odyssey-stock-long"
git cherry-pick <param/UI commit> <ODYSSEY.md commit>
git push wrgrant HEAD:odyssey-stock-long
```

Conflicts to expect: `honda.h` TX lists and `_initialize_*` ordering in `opendbc/sunnypilot/car/interfaces.py`. Before pushing, check that the catalog the new base reads (`openpilot/sunnypilot/models/fetcher.py`, `MODEL_URL`) still lists TSFDOM with a `minimum_selector_version` equal to `REQUIRED_JSON_VERSION` in `openpilot/sunnypilot/models/helpers.py`.

## Install

On the device (parked, van off), use the installer URL `installer.comma.ai/wrgrant/odyssey-stock-long`, or switch the git remote in `/data/openpilot`. The device builds from source on first boot.
