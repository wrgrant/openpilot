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
