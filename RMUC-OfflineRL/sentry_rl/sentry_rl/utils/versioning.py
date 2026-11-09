"""Version fingerprints for data, model and the action executor.

Changing the behaviour tree (or the Python executor that mirrors it) changes
the MDP.  Every rollout / dataset / checkpoint should carry these hashes so
numbers from different executor versions are never compared by accident.
"""
from __future__ import annotations

import hashlib
import json
import subprocess
from pathlib import Path

from ..config import resolve_path


def sha256_file(path: str | Path | None) -> str | None:
    if not path:
        return None
    path = Path(path)
    if not path.is_file():
        return None
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def git_rev(cwd: str | Path | None = None) -> str | None:
    try:
        out = subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=cwd,
                             capture_output=True, text=True, timeout=2.0)
    except (OSError, subprocess.SubprocessError):
        return None
    return out.stdout.strip() or None if out.returncode == 0 else None


def executor_fingerprint(cfg: dict) -> dict:
    """Hash the BT XML and the DecisionSwitch implementation, if present."""
    base = Path(cfg.get("_config_dir") or Path.cwd()).parent.parent
    bt = base / "behaviortree_27/rm_behavior_tree/rm_behavior_tree/config/rmul_2027_7v7.xml"
    switch = base / "behaviortree_27/rm_behavior_tree/rm_behavior_tree/plugins/control/decision_switch.cpp"
    return {
        "bt_xml": str(bt),
        "bt_xml_sha256": sha256_file(bt),
        "decision_switch": str(switch),
        "decision_switch_sha256": sha256_file(switch),
    }


def versions(cfg: dict, extra: dict | None = None) -> dict:
    payload = {
        "git": git_rev(Path(cfg.get("_config_dir") or Path.cwd()).parent.parent),
        "executor": executor_fingerprint(cfg),
        "obs_dim": int(cfg.get("_obs_dim", 0)) or None,
    }
    payload.update(extra or {})
    return payload


def dump_versions(cfg: dict, path: str | Path, extra: dict | None = None) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(versions(cfg, extra), ensure_ascii=False, indent=2,
                               sort_keys=True) + "\n", encoding="utf-8")
