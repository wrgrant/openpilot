# Odyssey EPS: firmware mod map and tools

Start here before working on the power-steering firmware of this van (2019 Odyssey Elite, EPS `39990-THR-A020`). It gathers what is known, where every piece lives, and what is still open. Nothing has been flashed on this van. The next step is reading its own EPS (see "Procedure").

## Words you'll see in the Torque Collective Discord

- **Torque Collective**: the Discord server where Honda EPS firmware work happens. Its channels are per car (odyssey-fw, crv-fw, civic-fw), plus general and fork-updates.
- **RWD**: Honda's firmware update file, the scrambled format the dealer tool flashes. `rwd.py` and nrdr's `rwd-xray-2026` unpack and rebuild it.
- **PTM (proper torque mod)**: an EPS firmware image with raised clamps, input scaling and loop gain, sometimes an added feedforward, and faster rate normalisation. It is firmware only. The software that drives it well is a separate piece, the Firmware Controller.
- **Firmware Controller / HondaEpsController**: openpilot-side controllers (nrdr and StarPilot names for the same idea) that invert the EPS's own law, both the command table and the rate damping. This fork's `OdysseyEpsMap` inverts only the command table.
- **Forks**: comma's openpilot → sunnypilot (this fork's base) → **nrdr** (Aragon [SP]'s Honda fork, branches `nrdr-clean`, `nrdr-nightly`, dated `nrdr-staging-*`, `nrdr-development`). Separately, FrogPilot → **StarPilot** (vote_for_nobody, "VFN", collaborates there). Jay DeFuria's Odyssey fork is `jasondefuria/openpilot_ody`.
- **Konik**: a third-party replacement for comma's cloud (device registration, route upload and viewing), at `stable.konik.ai`. The community uses it to share routes. This van needs no cloud; logs come over Tailscale.
- **Blendful**: a sunnypilot lateral feature that pulls the model's path toward lane centre when it trusts the lane lines. It is in this fork, off by default (`Blendful` param).
- **Variable / firmware-derived steer ratio**: Honda racks have a ratio that changes with wheel angle, but openpilot learns one number. nrdr reads the ratio table from the firmware instead. The THR-A020 image probably holds such a table too; it is not located yet.
- **Lateral stiction (NrdrLatStiction)**: nrdr's output stage that holds small commands to stop the EPS dithering, then lets decisive corrections through. It is a different answer to the same small-command problem that this fork's pulse idea attacks.
- **Pedal interceptor**: add-on gas-pedal hardware for Hondas without stop-and-go ACC. This van uses factory ACC and does not need it.

## What is known

**The unit.** Nidec EPS, SuperG SH2A processor; application at flash 0xC000, 0x74000 bytes. On this van the EPS answers diagnostics on CAN bus 0 through the camera harness (`carFw` in every drive log): UDS on 0x18DA30F1 / 0x18DAF130, CCP on 0x721 (requests) / 0x722 (replies). No OBD power cable is needed for that.

**How it steers** (`tools/odyssey_sim/reports/report_eps_law.md`, `ODYSSEY.md` "Measured facts"). Motor torque is a command map minus damping on the wheel's own turning rate, about 1.4-2.3 raw units per °/s above 15 mph. Commands under about 2-3% give nothing only while the wheel is still. Lane-keeping torque caps near 75 raw units, about 0.85 m/s² at 34 mph, and is lower below 15 mph. The open-source nrdr fork documents the same kind of law for the Clarity's Nidec EPS (`nrdr/features/lateral/vfn_eps_core.py`): the lane-keeping value becomes a target that the firmware tracks against a filtered steering rate.

**The image** (`rwd.py`). Three checks guard the application: a big-endian word sum stored at flash 0x7FF80, a negative word sum at 0x7FFFE, and a CRC-32/BZIP2 of flash 0x60000-0x6FF5F stored at 0x6FF7C. The flasher checks only the first two. A stale block CRC flashes fine and then faults the EPS (permanent fault, assist lost a second into the first steer). The torque table (inputs 0-1024, outputs 0, 2019 ... 4864) is stored six times, three copies inside the CRC block. Minimum-engagement-speed tables sit in banks at flash 0x57B00 + n × 0x300, 0.5 km/h per count, 10 counts stock; bank 7 (70 counts) looks diagnostic and is left alone.

**Identities.** Stock reports `39990-THR-A020`; a modified image reports `39990-THR,A020`; an erased or half-flashed EPS answers as its bootloader `39990-THR-A010`. A recovery image must list A010 in its header, or the flasher finds no key. Our candidates in `~/comma-logs/odyssey-eps/images/` do not list it yet.

**The community's stock image is not a Honda file.** It was assembled by hand from a bench dump of an unknown unit, and nobody has compared it byte for byte with a clean read of an Odyssey. Its block CRC is original and a 2018 Odyssey runs on it. Reading this van's EPS settles it.

**Community experience on THR-A020** (Torque Collective Discord, `#odyssey-fw`). One van is flashed: Jay DeFuria's 2018 Odyssey, on a 2.5x table-scaled image since 4 October 2026, after two images faulted from the stale block CRC. A flash started with openpilot still running left his EPS erased and answering as A010; a stock image listing A010 recovered it. Stock has gone back on every time; a flash takes about 52 s. No PTM image (firmware with a feed-forward code hook) has been driven on an Odyssey. vote_for_nobody, who writes the images, has offered to build a proper Odyssey PTM, as for the Acura RDX, whose firmware is very similar.

**Software side.** The comma sends the same `STEERING_CONTROL` message to stock, table-scaled and PTM images; detection is the comma in the identity. Controllers: this fork's `OdysseyEpsMap` (`opendbc/sunnypilot/car/honda/eps_mod.py`) inverts a table-scaled image so it starts out driving like stock; it does not undo the rate damping, and a PTM image changes the firmware's own loop, so it would need its own calibration. The community runs nrdr's "Firmware Controller" (called HondaEpsController in their starpilot branch), which models the firmware law and inverts it; it has no Odyssey profile yet. Jay's fork (`jasondefuria/openpilot_ody`) uses PID gains 0.3 / 0.1 for the modified Odyssey. Drive routes from modified cars go to Konik (`stable.konik.ai`) in that community, not comma connect.

## Procedure (planned; each live step needs an explicit go, someone at the van and a battery charger on)

Van parked, ignition on, engine off, charger connected, openpilot stopped on the comma (`sudo systemctl stop comma`), then:

1. **Read this van's EPS** with vote_for_nobody's CCP script (`ccp_honda_eps.py`, which reads with UPLOAD, not SHORT_UP, so the result is flash, not RAM through pointers). Review, exact command and offline dry run: `~/comma-logs/odyssey-eps/ccp/REVIEW.md`. Accept the read only if all three checks pass, then compare it with the community stock image. Keep it as this van's recovery image.
2. **Dry run of the flasher** (logs in to the EPS and aborts before erasing), on bus 0; the kit's default of bus 1 is wrong for this van.
3. **Flash stock onto stock** with an image that lists A010, then confirm assist and identity.
4. **Flash a modified image**, then a stock-equivalent drive before raising authority step by step.

## Where everything is

| What | Where |
|---|---|
| Codec, candidate builder, map fit, rate-law fit | `tools/odyssey_eps/` (`rwd.py`, `prepare.py`, `fit.py`, `eps_law.py`) |
| Flasher review and step-up plan; rate-law report | `tools/odyssey_sim/reports/report_eps.md`, `report_eps_law.md` |
| Stock and community 2.5x images (Honda binaries, never in git) | `~/comma-logs/odyssey-eps/dl/rwd/` |
| Our x1.5 / x2.0 candidates | `~/comma-logs/odyssey-eps/images/` (README there) |
| Reviewed flashing kit | `~/comma-logs/odyssey-eps/kit/` (README there) |
| CCP read script, its review and read checker | `~/comma-logs/odyssey-eps/ccp/`; where it was found: `ccp_upload_script_locations.md` |
| Discord notes | `~/comma-logs/odyssey-eps/discord_odyssey_fw_early_history.md` (13-26 Sept), `discord_odyssey_fw_2026-09-26_to_10-05.md`, `discord_server_pass_2026-11-10.md` (since October, all channels), `discord_export_report.md` (the server's older archive), raw messages in `discord-raw/` |
| Community code study (nrdr, starpilot, Jay's fork) | `~/comma-logs/odyssey-eps/ptm_starpilot_code_report.md`; clones in `repos/` |
| Rate-law plots and data | `~/comma-logs/odyssey-evidence/eps_law/` |
| Discord channels | Torque Collective `#odyssey-fw` https://discord.com/channels/376924440321327115/1548734864927498310 ; `#crv-fw` (CCP script) https://discord.com/channels/376924440321327115/1532127763434311750/1556829422282342413 |
| Public repos | https://github.com/nrdr/openpilot , https://github.com/nrdr/rwd-xray-2026 , https://github.com/jasondefuria/openpilot_ody , https://github.com/JamesL787/openpilot/tree/night-star7 , https://github.com/RiskyBiscuit-arc/eps-tools |
| Community firmware folder | https://drive.google.com/drive/folders/1Z4_axffop928ep2CAmaApmTjdKUQV4xW |

## Open questions

- Where the small-command threshold lives, and whether it is the damping law or a separate gate.
- What reduces torque below 15 mph. vote_for_nobody says the firmware's speed attenuation acts only at higher speeds; the damping (low-speed turns need fast wheel motion) may explain part of it.
- Whether a PTM image changes the rate damping, and what an Odyssey PTM does to the weave.
- The firmware's exact law: the pulse test's `pair` shots (an 8% pulse from rest against the same pulse on a reversal) separate the models.

# Tools (offline only)

`rwd.Image.decode(raw)` / `.encode()` validate the container sum, the two payload sums and the block CRC the EPS checks while running (flash 0x60000-0x6FF5F), the THR-A020 identity, keys, layout, torque axes and six table copies. `rwd.build(raw, outputs=..., limits=(...), identity=...)` or `scale=...` writes only known table/identity locations and recomputes checksums. Eight outputs exclude the leading zero knot. Cipher permutation is from nrdr-derived `check_rwd.py`; no downloaded module is imported.

Run from the repository with its Python:

```
.venv/bin/python tools/test_runner.py tools/odyssey_eps opendbc_repo/opendbc/sunnypilot/car/honda/test_honda.py
.venv/bin/python -m tools.odyssey_eps.fit --stock STOCK_NPZ_DIR --modified MODIFIED_NPZ_DIR --scale 2.5 --out NEW_OUTPUT_DIR
```

For firmware inputs use isolated Python (`-I`) and explicitly insert this trusted repository into sys.path before importing these modules. `prepare.main()` regenerates candidates only under `~/comma-logs/odyssey-eps/`. The binaries stay outside git. Reference tests skip when external files are absent; synthetic codec tests remain.

## Controller configuration

Modified Odyssey only (EPS identity `39990-THR,A020`). The `OdysseyEpsMap` param (JSON, registered in `params_keys.h`) is read once when the car controller is built; change it offroad. Example:

```json
{"firmware_scale":1.5,"authority_ceiling":1.0,"speed_bp":[0,40],"speed_gain":[1,1],"torque_bp":[0,1],"torque_v":[0,1]}
```

The firmware's torque table output acts as a virtual torque on the wheel, which then goes through the normal assist curve: across the command range and above 18 mph, the hand torque that produces the same motor torque is a constant 2400-2700 sensor units times the table output. So a modified image is matched to stock by inverting its table, and `opendbc.sunnypilot.car.honda.eps_mod.OdysseyEpsMap` does that:

1. The controller's torque goes through `torque_bp`/`torque_v` (or `torque_rows`, one row per speed breakpoint) to a stock-equivalent command. Values above 1 ask for more than stock can give, e.g. bp `[0,.8,1]`, v `[0,.8,1.5]` keeps the normal range as stock and adds authority only near the top.
2. That goes through the stock table to the virtual torque stock would produce, relative to stock's maximum, and is multiplied by `speed_gain` (to undo the low-speed fade).
3. It is capped at `authority_ceiling` (default 1 = stock's maximum, at most 2) and at what the flashed image holds.
4. The command sent is the one whose modified-table output equals that virtual torque.

With the defaults, a modified EPS receives the table output a stock one would, so it should drive like stock; that still has to be confirmed on the van. All images share one identity, so `firmware_scale` (1.5, 2 or 2.5) must match the flashed image. A missing or invalid config assumes 2.5, which sends the smallest commands.

The normalized rate limit stays 3/s. A second bound of `4096 * 0.03 / scale` counts per frame applies after the map, so speed gain and lookup slope cannot raise the command rate. An unmodified Odyssey's commands are unchanged, and no panda safety limit is relaxed.

## Offline characterization

Use actual CAN command caches (`cmd`, `motor`, `v`, `active`, `pressed`, `drv`, 100 Hz). `fit` keeps 1-second steady blocks, hands-off and |driver|<100, reports counts and 10th/90th percentile motor per speed/command bin, then inverts the measured response. It rejects insufficient coverage, plateaus, large monotonicity violations and out-of-range targets; it emits a candidate JSON only with at least two populated speed bins. No post-flash data exists yet. A short drive can fail to provide enough coverage; do not extrapolate missing data or force saturation to fill bins. The existing suite's small highway pulses measure onset and delay, not full-range authority or low-speed fade.

The fit pools motor magnitude across steering directions; review left/right held pulses separately with `tools/odyssey_eps_report.py` before accepting a symmetric map. Curve fits are observational, not proof of steering dynamics or override thresholds. Treat bin-to-bin interpolation as provisional and keep operation in covered speeds.

## Map of this addition

`rwd.py`: codec and exact rebuild seam. `prepare.py`: external candidate writer / calibration inventory. `fit.py`: cache characterization / conservative inverse-map seam. Tests exercise these seams and the Honda controller's actual transmitted torque bytes. Runtime mapping lives in the existing `opendbc.sunnypilot.car.honda.eps_mod` extension and is called after Honda's original rate limit.
