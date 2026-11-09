"""Sentry RL training workspace for RMUC 2027 (Arena-1)."""
from __future__ import annotations

import sys
from pathlib import Path

__version__ = "0.1.0"

# The workspace keeps an optional local dependency directory (CPU torch and its
# small runtime deps).  Putting it on sys.path here means every
# ``python -m sentry_rl...`` command works without exporting PYTHONPATH.
_DEPS = Path(__file__).resolve().parents[1] / ".deps"
if _DEPS.is_dir() and str(_DEPS) not in sys.path:
    sys.path.insert(0, str(_DEPS))
