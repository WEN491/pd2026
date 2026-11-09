#!/usr/bin/env bash
# One-command launcher for the sentry_rl workspace.
#
#   ./scripts/launch.sh            # default: tests + mock demo + real Arena-1 30 s
#   ./scripts/launch.sh train      # mock collect -> BC -> PPO -> eval
#   ./scripts/launch.sh arena      # real Arena-1 match (30 s, optional checkpoint)
#   ./scripts/launch.sh setup      # check/install torch + build Arena-1 only
#
# Env overrides: EPISODES, BC_EPOCHS, N_ENVS, PPO_STEPS, DURATION, CHECKPOINT.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPO="$(cd "${ROOT}/.." && pwd)"
MODE="${1:-demo}"

export CARGO_HOME="${REPO}/.scratch/rust/cargo"
export RUSTUP_HOME="${REPO}/.scratch/rust/rustup"
export PATH="${CARGO_HOME}/bin:${PATH}"

cd "${ROOT}"

need_torch() {
  if ! python3 - <<'PY'
import sys
sys.path.insert(0, ".")
import sentry_rl  # adds .deps to sys.path
import torch
print(f"torch {torch.__version__} (cuda={torch.cuda.is_available()})")
PY
  then
    cat >&2 <<'MSG'
[launch] torch is missing.
Install CPU torch into the workspace:
  python3 -m pip install --target .deps "torch==2.1.2+cpu" \
    --index-url https://download.pytorch.org/whl/cpu
  python3 -m pip install --target .deps filelock networkx fsspec "typing_extensions>=4.8"
or run: ./scripts/launch.sh setup
MSG
    exit 1
  fi
}

ensure_arena() {
  local bin="${REPO}/Arena-1/simulation/target/release/rmuc-sim"
  if [[ -x "${bin}" ]]; then
    echo "[launch] Arena-1 engine: ${bin}"
    return
  fi
  if ! command -v cargo >/dev/null 2>&1; then
    echo "[launch] cargo missing; cannot build Arena-1" >&2
    exit 1
  fi
  echo "[launch] building Arena-1 engine ..."
  cargo build --release --manifest-path "${REPO}/Arena-1/simulation/Cargo.toml"
}

case "${MODE}" in
  setup)
    need_torch
    ensure_arena
    echo "[launch] setup ok"
    ;;
  train)
    need_torch
    python3 -m sentry_rl.data.collect --config configs/sentry_2027.yaml --mock \
      --episodes "${EPISODES:-8}" --out runs/data/bc_mock.npz
    python3 -m sentry_rl.train.train_bc --config configs/sentry_2027.yaml \
      --data runs/data/bc_mock.npz --out checkpoints/bc.pt \
      --epochs "${BC_EPOCHS:-1}"
    python3 -m sentry_rl.train.train_ppo --config configs/sentry_2027.yaml \
      --init checkpoints/bc.pt --out checkpoints/ppo.pt --mock \
      --n-envs "${N_ENVS:-4}" --total-steps "${PPO_STEPS:-336}"
    python3 -m sentry_rl.eval.evaluate --config configs/sentry_2027.yaml \
      --checkpoint checkpoints/ppo.pt --mock --seeds 3
    ;;
  arena)
    need_torch
    ensure_arena
    extra=()
    if [[ -n "${CHECKPOINT:-}" ]]; then extra+=(--checkpoint "${CHECKPOINT}"); fi
    python3 scripts/run_arena_smoke.py --duration "${DURATION:-30}" "${extra[@]}"
    ;;
  demo)
    need_torch
    ensure_arena
    python3 -m pytest tests -q
    python3 -m sentry_rl.envs.mock
    extra=()
    if [[ -n "${CHECKPOINT:-}" ]]; then extra+=(--checkpoint "${CHECKPOINT}")
    elif [[ -f checkpoints/ppo.pt ]]; then extra+=(--checkpoint checkpoints/ppo.pt); fi
    python3 scripts/run_arena_smoke.py --duration "${DURATION:-30}" "${extra[@]}"
    ;;
  *)
    echo "usage: $0 [demo|train|arena|setup]" >&2
    exit 2
    ;;
esac
