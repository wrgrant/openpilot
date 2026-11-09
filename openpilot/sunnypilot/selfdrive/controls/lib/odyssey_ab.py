"""Odyssey road A/B tests: rotate between arms in fixed blocks of the monotonic clock (the clock behind logMonoTime), so
one drive compares them on the same roads and tools/odyssey_wander.py --ab can tell afterwards which arm ran when."""
AB_BLOCK_S = 60.0  # each block holds three or four of the 15-20 s swings the wander score measures

DEADBAND_ARMS = ("off", "kick", "bump")  # SteerDeadbandCompAB
NNLC_ARMS = ("on", "off")  # NnlcAB


def ab_arm(mono_s: float, arms: tuple[str, ...]) -> str:
  return arms[int(mono_s // AB_BLOCK_S) % len(arms)]
