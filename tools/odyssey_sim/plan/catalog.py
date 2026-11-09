"""Read-only bundle availability audit; never load third-party executable pickles."""
import argparse
import hashlib
import json
from pathlib import Path
from urllib.request import urlopen


BASE = 'https://raw.githubusercontent.com/sunnypilot/sunnypilot-models/gh-pages/docs/'


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--scratch', required=True, type=Path)
  a = ap.parse_args()
  result = []
  for name, source in [('driving_models_v25.json', 'qcom'), ('driving_models_chestnut_v28.json', 'chestnut')]:
    raw = urlopen(BASE+name, timeout=30).read()
    (a.scratch/name).write_bytes(raw)
    for b in json.loads(raw)['bundles']:
      artifacts = [m['artifact'] for m in b['models']]
      portable = any(x['file_name'].endswith('.onnx') for x in artifacts)
      result.append({'name': b['display_name'], 'ref': b['ref'], 'runner': b['runner'], 'generation': b['generation'],
                     'source': source, 'minimum_selector_version': b['minimum_selector_version'],
                     'branch_selector_compatible': int(b['minimum_selector_version']) == 21,
                     'mac_onnx_available': portable, 'artifacts': artifacts,
                     'catalog_sha256': hashlib.sha256(raw).hexdigest(),
                     'input_contract': 'compiled runner ABI; no portable ONNX graph contract in bundle'})
  (a.scratch/'catalog_audit.json').write_text(json.dumps(result, indent=2))
  print(len(result), 'bundles;', sum(x['mac_onnx_available'] for x in result), 'portable ONNX')


if __name__ == '__main__':
  main()
