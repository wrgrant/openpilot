"""Reproducible Odyssey inverse-model experiments; training dependencies stay off-device."""

from pathlib import Path

SCRATCH = Path('/private/tmp/claude-501/-Users-williamgrant-Documents-My-vault/ae63c39a-5411-41d2-abc4-99cb7e2003e6/scratchpad')
MODEL_DIR = Path(__file__).resolve().parents[3] / 'openpilot/sunnypilot/neural_network_data/neural_network_lateral_control'
