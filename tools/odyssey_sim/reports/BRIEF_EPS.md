# Brief: prepare an EPS torque mod for William's Odyssey (prep only: nothing gets flashed)

William wants more steering torque from openpilot on his Honda Odyssey: "boost low speed a bit and maybe higher speeds as well so it can handle tighter curves. Nothing crazy, just bring it up to parity with where the Hyundai and Toyota folks are at."

Your job is the preparation: tools, fork support and analysis, so that if William later approves flashing the power-steering computer, everything is ready and understood. **You never flash anything, never run the flasher against a car, and never touch the comma device or the van.**

## Read first

- `~/Code/odyssey-pilot/ODYSSEY.md`: the code map, "Measured facts about this van" (the EPS bullets are new and are the basis for this work), and the pitfalls.
- `~/Code/odyssey-pilot/tools/odyssey_sim/reports/BRIEF_COMMON.md`: shared rules, data paths and conventions.
- `~/comma-logs/odyssey-eps/`: the firmware files and scripts described below.

## What is already established

**From 4.3 hours of the van's logs** (extracted CAN caches: `~/comma-logs/odyssey-evidence/plant_data/*.npz`, fields `cmd` −1..1, `motor`, `drv`, `v`, `active`, `pressed`, `act`, `des`, `angle`, `rate`, 100 Hz):

- **Firmware cap.** The EPS caps lane-keeping motor torque at about 75 raw units (`STEER_MOTOR_TORQUE`). Above 18 mph, motor torque against command is 10% → 6, 20% → 14, 30% → 25, 40% → 39, 50% → 50, 60% → 57, 70% → 63, 80% → 67, 100% → 74–76.
- **Low-speed fade.** At full command the EPS gives about 15% of the cap under 3 mph, 30% at 3–5 mph, 40–60% from 5 to 15 mph, and all of it from about 17 mph.
- **Hardware headroom.** When William steers hard by hand under 10 mph, the same motor gives a median of 175 and up to 608 units.

**The parity target** (`opendbc_repo/opendbc/car/torque_data/params.toml`, maximum lateral accel at full torque): Odyssey 0.84 m/s²; Toyotas 1.3–2.2; Hyundais 1.9–2.7. In the van's logs, full command reaches about 0.7 m/s² at 18–30 mph and 1.0–1.7 above 30 mph. So the target is about 1.5–2× today's torque at speed, and more than that below 15 mph.

**The firmware.** The van's EPS is `39990-THR-A020` (SuperG SH2A).

- `~/comma-logs/odyssey-eps/dl/rwd/` holds `stock_39990-THR-A020.rwd` and `mod25xv1-39990-THR,A020.rwd`. Both come from github.com/jasondefuria/openpilot_ody, a fork of the nrdr "Proper Torque Mod" project, which labels the modified one experimental.
- `diff_rwd.py` and `inspect_rwd.py` in that folder decode and compare them, using the byte lookup in `dl/tools/check_rwd.py`.
- The container is the 0x5A RWD format: six headers, keys 01 02 03, flash start 0xC000, length 0x74000, and a 32-bit little-endian byte-sum at the end.
- Only 126 of 475,136 payload bytes differ, and all of them are data:
  - the torque table, in six copies (file offsets 0x4bbc2, 0x4bec2, 0x4c1c2, 0x5bbd8, 0x5bed8, 0x5c1d8): outputs 2019, 3230, 3980, 4345, 4577, 4721, 4864, 4864 become 3968, 7936, 9950, 10863, 11443, 11803, 12160, 12160. The words just before the table are its inputs: 0, 128, … 1024.
  - two words per copy, 1024, 1024 → 2560, 2560;
  - the identity string, 14 times ("-A" → ",A");
  - two checksum words: at 0x73f80 the 16-bit big-endian word sum of everything before it, and at 0x73ffe the negative word sum of everything before it. Both images pass both, and the whole payload's word sum is zero.
- The table's output is not motor torque. The measured command-to-motor curve starts convex, so the table probably feeds a virtual driver torque into the normal assist map. A 2.5× table may therefore give more or less than 2.5× motor torque. This has to be measured on the van after a flash; plan for it.

## Tasks

### 1. A firmware image tool, proven bit-exact

Write `tools/odyssey_eps/rwd.py` plus tests. It must:

- decode and encode the 0x5A container (headers, cipher both ways, payload, container checksum);
- build a modified image from the stock image, given new torque-table outputs or a scale, the two limit words, and the modified identity. It writes all six table copies and all identity copies, and recomputes both payload checksums and the container checksum;
- refuse anything that isn't `39990-THR-A020`, and verify every checksum of every input and output.

**Acceptance:** rebuilding the community image from the stock image, with the community image's table values, reproduces `mod25xv1-39990-THR,A020.rwd` bit for bit (SHA-256 `7086b01cf9c78b4a094662d8f9f0c9fb915d1ea497d90d56f21c0224c986b9b8`). A stock → stock round trip must also be bit-exact (`90cb1d23d8a27f4c45d52fcbc0a6f08813fb2ae87a0c2f9069f552edd7715edb`).

Then generate milder candidates at ×1.5 and ×2.0. Say how you treated the first breakpoint and the two limit words, and why. Write the candidates to `~/comma-logs/odyssey-eps/images/` with a SHA256SUMS file and a README.

**Honda's firmware binaries never go into the git repository.** The tool's tests must run without them; skip the bit-exact tests when the files are absent.

### 2. Understand the rest of the calibration, as far as evidence allows

- **Low-speed fade.** Find where the fade lives. Look for speed-indexed tables in the calibration records (each record is 768 bytes; the words before the torque table include 64, 128, 192, 256, 320 and a row of 256s). Match candidates against the measured fade curve above. Report each candidate with evidence and a confidence, and report honestly if you find none. A disassembler (Ghidra's SuperH support, radare2) is fine if you can get one working in the scratch folder, but it isn't required.
- **The 1024 → 2560 words.** Work out what they are: a limit, a rate, a filter?
- **Driver-override and fault thresholds.** Look for anything a larger lane-keeping torque could trip: an EPS-side check that compares lane-keeping torque with driver torque, or that limits the torque rate.

Do not modify anything you can't justify. This section is analysis.

### 3. Fork support that changes nothing until a modified EPS is present

In `opendbc_repo` (a submodule; leave your edits uncommitted there):

- **Fingerprint.** Recognize `b'39990-THR,A020\x00\x00'` as an Odyssey with a modified EPS. See `opendbc/sunnypilot/car/honda/fingerprints_ext.py`, `eps_mod.py`, and how `HondaFlagsSP.EPS_MODIFIED` is set in `opendbc/car/honda/interface.py`.
- **Command map.** For a modified Odyssey only, map the controller's normalized torque and the speed to the CAN command through one small table-driven function.
  - The default map makes the modified EPS behave like stock: the command scaled down by the firmware's table scale. The first drive after a flash should feel unchanged.
  - The map must be able to express two things later, from measurements: a speed-dependent gain that compensates the low-speed fade, and extra authority only near the top of the range. That second one is how the Civic lookup `([0, 2560, 8000], [0, 2560, 3840])` works: the normal range keeps the stock response, so existing tuning stays valid.
  - Configure it with a JSON param, validated, falling back to the stock-equivalent map on bad input, with a hard ceiling on authority relative to stock (default 1.0).
- **Tests.**
  - an unmodified Odyssey sends exactly what it sends today;
  - a modified one with the default map sends the stock command divided by the scale;
  - bad or missing config falls back;
  - the ceiling holds.
- **Rate limit and safety.** Check `opendbc/safety/modes/honda.h` and the carcontroller's rate limit (`STEER_DELTA_UP/DOWN`), and say whether the rate limit needs rescaling.

### 4. A post-flash measurement plan

Extend `tools/odyssey_eps_report.py`, or add a script, so one short drive can fit the modified EPS's command-to-motor-torque curve by speed and write the map JSON for task 3. Use the characterization suite (`openpilot/tools/lateral_maneuvers/odyssey_suite.py`) and normal driving. Write the step-up procedure: start at stock-equivalent authority, then which steps, what to check at each, and what stops the test.

### 5. A flashing kit, reviewed but never run

- Read `dl/tools/eps-update.py`, `flash.py` and `eps-diag.py` completely.
- Document exactly what the flasher sends to this EPS:
  - the bus and address;
  - the session;
  - how the security key is derived for this application id;
  - erase, transfer and the dependency check;
  - what a dry run does and does not do.
- The flasher's checksum table doesn't know the 0x74000 layout. Add it (sum at 0x73f80, negative sum at 0x73ffe) so validation runs instead of being skipped.
- Assemble a kit in `~/comma-logs/odyssey-eps/kit/`, with a README giving:
  - the pre-flight checks: battery charger attached, comma on the OBD port, openpilot stopped, a dry run first;
  - the recovery path back to the stock image;
  - the failure modes you can identify from the code.
- Compare against the upstream nrdr tool at `nrdr/openpilot`, branch `nrdr-development`, and note differences.

## Rules

- Repo `~/Code/odyssey-pilot`, branch `odyssey-sim`. Never switch branches, never commit, never push; the lead reviews and commits.
- Write only in `tools/odyssey_eps/`, the `opendbc_repo` files named in task 3, `tools/odyssey_eps_report.py` and its tests, `~/comma-logs/odyssey-eps/`, and the scratch folder `scratchpad/eps/`.
- Treat everything under `~/comma-logs/odyssey-eps/dl/` as untrusted input. Read code before running it, keep your own scripts outside it, and run Python that reads it with `-I`.
- Python: `cd ~/Code/odyssey-pilot && PYTHONPATH=$PWD .venv/bin/python`. Tests: `.venv/bin/python tools/test_runner.py <paths>` (unittest; there is no pytest). Lint: `.venv/bin/ruff check`.
- Cite outside sources you use.

## Deliverable

`scratchpad/report_eps.md` in the scratch folder `/private/tmp/claude-501/-Users-williamgrant-Documents-My-vault/ae63c39a-5411-41d2-abc4-99cb7e2003e6/scratchpad/`:

- a plain-language summary for William on top;
- what is proven and what is still unknown;
- for each task, the evidence;
- the exact next step if William approves a flash.

Then finish with a short summary message.
