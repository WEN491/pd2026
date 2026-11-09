"""Python macro executor for the Arena-1 training env.

This is the training-side twin of the ROS behaviour tree
(``behaviortree_27/.../rmul_2027_7v7.xml``): the same 7 intents, target points,
postures, target selection and spin rule.  Keeping the two in sync is a
versioned contract; see ``utils/versioning.py``.
"""
from __future__ import annotations

import math

from .. import parse
from ..actions import Intent
from ..config import goal


class MacroExecutor:
    def __init__(self, cfg: dict, sdk=None):
        self.cfg = cfg
        if sdk is None:
            from rmuc_arena import sdk as arena_sdk  # lazy: no Arena-1 in unit tests
            sdk = arena_sdk
        self.sdk = sdk

    # ------------------------------------------------------------------ helpers
    def _goal_for(self, intent: int) -> list[float]:
        """Config goals are centre-origin (the behaviour tree's frame).

        Arena-1 plays on a corner-origin 0..W x 0..H field, so a goal has to be
        shifted by the field centre; on the blue side it is also point-mirrored.
        """
        key = {
            Intent.HOLD_OUTPOST: "hold_outpost",
            Intent.HOLD_BASE: "hold_base",
            Intent.TAKE_OUTPOST: "take_outpost",
            Intent.TAKE_BASE: "take_base",
            Intent.RESUPPLY: "resupply",
            Intent.RETREAT: "retreat",
            Intent.ENGAGE: "hold_outpost",  # engage overrides the move with PursueEnemy
        }[Intent(intent)]
        x, y = goal(self.cfg, key)
        if str(self.cfg.get("team", "red")) != "red":
            x, y = -x, -y
        field = self.cfg["field"]
        return [x + float(field["center_x"]), y + float(field["center_y"])]

    def _posture_for(self, intent: int, robot: dict) -> str:
        hp = parse.hp_ratio(robot)
        if intent == Intent.HOLD_OUTPOST:
            return "defense" if hp < 0.5 else "attack"
        return {
            Intent.HOLD_BASE: "defense",
            Intent.TAKE_OUTPOST: "attack",
            Intent.TAKE_BASE: "attack",
            Intent.RESUPPLY: "move",
            Intent.ENGAGE: "attack",
            Intent.RETREAT: "defense",
        }[Intent(intent)]

    def _spin_for(self, intent: int) -> float:
        return 0.0 if Intent(intent) == Intent.RESUPPLY else 3.0

    # ----------------------------------------------------------------- command
    def command(self, intent: int, observation: dict, tick: int = 0) -> dict:
        """Return one 10 Hz Arena-1 action for the sentry."""
        intent = int(intent)
        robot = parse.own_robot(observation, self.cfg.get("own_slot", "7"))
        if not robot:
            return {"move": [0.0, 0.0], "fire": False}
        arena = observation.get("arena") or {}

        target = None
        if intent in (Intent.HOLD_OUTPOST, Intent.HOLD_BASE, Intent.TAKE_OUTPOST,
                      Intent.TAKE_BASE, Intent.ENGAGE, Intent.RETREAT):
            target = self.sdk.nearest_enemy(robot, observation)

        aim = None
        pitch = 0.04
        fire = False
        if target is not None:
            try:
                aim = float(self.sdk.aim_at(robot, target))
                pitch = float(self.sdk.aim_pitch(robot, target))
                fire = bool(self.sdk.can_fire(robot)) and int(tick) % 2 == 0
            except (KeyError, TypeError, ValueError):
                aim, fire = None, False

        if Intent(intent) == Intent.ENGAGE and target is not None:
            point = [float(target.get("x", robot.get("x", 0.0))),
                     float(target.get("y", robot.get("y", 0.0)))]
        else:
            point = self._goal_for(intent)
        try:
            move = self.sdk.navigate(robot, point, arena) if arena else (0.0, 0.0)
        except (KeyError, TypeError, ValueError):
            move = (0.0, 0.0)

        posture = self._posture_for(intent, robot)
        spin = self._spin_for(intent)
        return self.sdk.action(
            move=move,
            aim=aim,
            fire=fire,
            pitch=pitch,
            posture=posture,
            posture_enhanced=False,
            yaw_rate=spin,
        )

    def describe(self) -> str:
        return f"MacroExecutor(goals={sorted(self.cfg['goals'])})"
