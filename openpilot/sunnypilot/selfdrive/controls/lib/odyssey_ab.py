"""Odyssey road A/B tests: rotate between arms in fixed blocks of the monotonic clock (the clock behind logMonoTime), so
one drive compares them on the same roads and tools/odyssey_wander.py --ab can tell afterwards which arm ran when."""
AB_BLOCK_S = 60.0  # each block holds three or four of the 15-20 s swings the wander score measures

WANDER_BLOCK_S = 180.0  # wander needs long blocks: score the last 150 s of each (the wander worker's A/B plan)

DEADBAND_ARMS = ("off", "kick", "bump")  # SteerDeadbandCompAB
NNLC_ARMS = ("on", "off")  # NnlcAB
WANDER_ARMS = ("off", "inverse")  # OdysseyWanderAB
# NnlcModelAB: stock HONDA_ODYSSEY NNLC vs the van-specific models (report_nnlc_smooth.md), WANDER_BLOCK_S blocks
NNLC_MODEL_ARMS = ("stock", "S3", "S5")
NNLC_MODEL_FILES = {"S3": "HONDA_ODYSSEY_WILLIAM_S3.json", "S5": "HONDA_ODYSSEY_WILLIAM_S5.json"}


def ab_arm(mono_s: float, arms: tuple[str, ...], block_s: float = AB_BLOCK_S) -> str:
  return arms[int(mono_s // block_s) % len(arms)]
