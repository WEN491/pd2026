PY ?= python3
CONFIG ?= configs/sentry_2027.yaml
CARGO_HOME ?= $(CURDIR)/../.scratch/rust/cargo
RUSTUP_HOME ?= $(CURDIR)/../.scratch/rust/rustup
export CARGO_HOME RUSTUP_HOME
PATH := $(CARGO_HOME)/bin:$(PATH)
export PATH

.PHONY: up test mock collect bc ppo eval smoke arena-build arena-smoke pool-list pool-smoke pool-exam clean

up:
	./scripts/launch.sh demo

test:
	$(PY) -m pytest tests -q

mock:
	$(PY) -m sentry_rl.envs.mock

collect:
	$(PY) -m sentry_rl.data.collect --config $(CONFIG) --mock --episodes 8 \
		--out runs/data/bc_mock.npz

bc:
	$(PY) -m sentry_rl.train.train_bc --config $(CONFIG) \
		--data runs/data/bc_mock.npz --out checkpoints/bc.pt

ppo:
	$(PY) -m sentry_rl.train.train_ppo --config $(CONFIG) \
		--init checkpoints/bc.pt --out checkpoints/ppo.pt --mock

eval:
	$(PY) -m sentry_rl.eval.evaluate --config $(CONFIG) \
		--checkpoint checkpoints/ppo.pt --mock --seeds 5

smoke:
	$(PY) -m sentry_rl.data.collect --config $(CONFIG) --mock --episodes 8 \
		--out runs/data/bc_mock.npz
	$(PY) -m sentry_rl.train.train_bc --config $(CONFIG) \
		--data runs/data/bc_mock.npz --out checkpoints/bc.pt --epochs 1
	$(PY) -m sentry_rl.train.train_ppo --config $(CONFIG) \
		--init checkpoints/bc.pt --out checkpoints/ppo.pt --mock \
		--n-envs 4 --total-steps 336
	$(PY) -m sentry_rl.eval.evaluate --config $(CONFIG) \
		--checkpoint checkpoints/ppo.pt --mock --seeds 3

arena-build:
	cargo build --release --manifest-path ../Arena-1/simulation/Cargo.toml

arena-smoke:
	$(PY) scripts/run_arena_smoke.py --duration 30

POOL_ARGS ?= --checkpoint checkpoints/ppo.pt --opponents 4 --duration 420

pool-list:
	$(PY) -m sentry_rl.pools --held-out

pool-smoke:
	$(PY) scripts/eval_pool.py --opponents 1 --duration 60 --no-swap \
		--out runs/eval/pool_smoke.json --pool-out runs/pools/pool_smoke.json

pool-exam:
	$(PY) scripts/eval_pool.py $(POOL_ARGS)

clean:
	rm -rf runs checkpoints .pytest_cache
	find sentry_rl tests -type d -name __pycache__ -prune -exec rm -rf {} +
