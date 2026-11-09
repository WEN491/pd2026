# sentry_rl — RMUC 2027 哨兵 RL 训练工作区

这是一个**独立于离线 RL 框架**的训练工作区：目标不是学底层走位/瞄准，而是学
“每 5 s 在 7 个宏意图里选一个”，具体动作由行为树 / Python 执行器完成。

对应计划：[`docs/plans/arena1-sentry-rl.md`](../docs/plans/arena1-sentry-rl.md)；
行为树执行器：[`behaviortree_27/.../config/rmul_2027_7v7.xml`](../behaviortree_27/rm_behavior_tree/rm_behavior_tree/config/rmul_2027_7v7.xml)。

## 一键拉起

```bash
cd sentry_rl
./scripts/launch.sh            # 测试 + mock 一局 + 真实 Arena-1 30 s
./scripts/launch.sh train      # mock 采集 -> BC -> PPO -> 评估
./scripts/launch.sh arena      # 真实 Arena-1（可选 checkpoint）
./scripts/launch.sh setup      # 只检查 torch / 构建 Arena-1
```

或 `make up`。默认会自动找 `.deps` 里的 torch 和 `Arena-1/simulation/target/release/rmuc-sim`；
缺引擎时才重新 `cargo build`。

## 目录

```
sentry_rl/
  configs/sentry_2027.yaml   # 所有参数
  sentry_rl/
    envs/                    # obs / radar / reward / mask / Arena-1 adapter / mock
    pools.py                 # 队友池 / 对手池（Arena-1 examples，可冻结）
    models/policy.py         # 8 万参数 MLP + masked categorical
    algos/bc.py, ppo.py      # BC 预训练 + PPO
    train/                   # 训练 CLI
    data/                    # 标签器 / 数据收集
    eval/                    # 评估协议
    utils/                   # 配置、种子、版本
  scripts/                   # launch.sh / run_arena_smoke.py / eval_pool.py
  tests/                     # 纯 numpy 部分可离线跑；torch 测试自动 skip
```

## 安装

**本机已配置**：CPU torch 2.1.2 装在 `sentry_rl/.deps/`（约 700 MB），
`sentry_rl/__init__.py` 会自动把它加入 `sys.path`。因此在本目录直接运行
`python -m sentry_rl...` / `make ...` 即可，不需要 export `PYTHONPATH`。

换一台机器时：

```bash
cd sentry_rl
# 1) CPU torch（有 CUDA 就换成对应 wheel）
python -m pip install --target .deps "torch==2.1.2+cpu" \
  --index-url https://download.pytorch.org/whl/cpu
# 2) torch 的少量运行时依赖
python -m pip install --target .deps filelock networkx fsspec "typing_extensions>=4.8"
# 3) 可选：把本包也装上（不装也能从当前目录运行）
python -m pip install -e . --no-deps
python -m pytest tests -q
```

Arena-1 需要 Rust 工具链和引擎二进制（本机已构建）：

```bash
# 本机 Rust 工具链在仓库根的 .scratch/rust（没有全局 cargo）
export CARGO_HOME="$(pwd)/../.scratch/rust/cargo"
export RUSTUP_HOME="$(pwd)/../.scratch/rust/rustup"
export PATH="$CARGO_HOME/bin:$PATH"
cargo build --release --manifest-path ../Arena-1/simulation/Cargo.toml
python scripts/run_arena_smoke.py --duration 30
# 用训练好的 checkpoint 在真实 Arena-1 里跑：
python scripts/run_arena_smoke.py --duration 30 --checkpoint checkpoints/ppo.pt
```

## 快速开始（不依赖 Arena-1）

Mock 环境把 obs/mask/reward/终止条件整条链路跑通，用来验证工作区：

```bash
python -m pytest tests -q
python -m sentry_rl.envs.mock          # 跑 1 局并打印
```

## 训练

### 1. 收集 BC 数据

```bash
# Mock：只验证数据格式
python -m sentry_rl.data.collect --config configs/sentry_2027.yaml \
  --mock --episodes 8 --out runs/data/bc_mock.npz

# Arena-1：用示例策略当“脚本哨兵”收集
python -m sentry_rl.data.collect --config configs/sentry_2027.yaml \
  --red-teammates ../Arena-1/examples/claude_opus55_xhigh_2027_team \
  --blue ../Arena-1/examples/claude_opus55_xhigh_2027_hold \
  --episodes 50 --out runs/data/bc_arena.npz
```

### 2. BC 预训练

```bash
python -m sentry_rl.train.train_bc --config configs/sentry_2027.yaml \
  --data runs/data/bc_arena.npz --out checkpoints/bc.pt
```

### 3. PPO 微调

```bash
python -m sentry_rl.train.train_ppo --config configs/sentry_2027.yaml \
  --init checkpoints/bc.pt --out checkpoints/ppo.pt --mock
```

`--mock` 用 Mock 环境跑通循环；去掉它需要 Arena-1 + 队友/对手策略。

### 4. 评估 / 导出

```bash
python -m sentry_rl.eval.evaluate --config configs/sentry_2027.yaml \
  --checkpoint checkpoints/ppo.pt --mock --seeds 20
python -m sentry_rl.export --config configs/sentry_2027.yaml \
  --checkpoint checkpoints/ppo.pt --out checkpoints/sentry_policy.ts
```

## 队友池 / 对手池

Arena-1 的 `examples/` 本身就是策略库：2027 赛季有 15 个可用的整队脚本（`mode: team`）
和 72 个单机脚本（`mode: adaptive`，按机型注册，`act()` 只处理 `obs['controlled_ids']`
里的槽位）。`sentry_rl.pools` 把它们读成不可变条目（`name@version` + 源码 sha256）：

| 组件 | 作用 |
|---|---|
| `TeammatePool` | `adaptive` 条目，按槽位给队友 1/3/4/6 各取一个 |
| `OpponentPool` | `team` 条目，作为对面整队 |
| `split_held_out` | 按 hash 确定性切 25% 当"考卷外"对手，用于测泛化 |
| `save_pool` / `load_pool` | 冻结成 JSON（带 digest），事后能复现当时那一套 |

`mix=homo` 让四个队友同倾向（balanced/assault/guard），`mix=hetero` 每槽独立抽——
后者才是真正逼配合的设置。池条目用 manifest 的 `version` 做键，改过源码的策略
`version` 会变，胜率不会跨版本污染。

```bash
make pool-list        # 列出现有池与留出对手
make pool-smoke       # 1 对手 × 60 s，验证链路
make pool-exam        # 4 对手 × 1 种子 × 换边 × 420 s，约 20 分钟

# 等价的手写命令
python scripts/eval_pool.py --checkpoint checkpoints/ppo.pt --opponents 4 \
  --opponent claude_opus55_xhigh_2027_team --opponent gpt61_2027_pressure \
  --opponent gemini38_2027_tech_overload --opponent step5_2027_echo
python scripts/eval_pool.py --held-out --opponents 5      # 测留出对手
```

考卷写 `runs/eval/pool_exam.json`：逐局 `winner / control / sentry_hp / actions`，
加按边、按对手的胜率与控制度聚合；报告里带 checkpoint sha256 与 pool digest。

两个约定：

- **`team` 模式脚本只当对手用**。把它削成 1/3/4/6 四个槽的队友，会丢掉它原本给
  哨兵的走位计划，行为和整队出场不一致；队友侧优先用 `adaptive` 按槽组合。
- 一场 420 s 真实对局约 2.5 分钟墙钟（进程内策略 + 每步 JSON IPC），所以换池评估
  比训练更贵；训练侧的池采样要配并行 actor 才有意义。

## 关键约定

- **决策步长 5 s**：一局 84 步；底层 10 Hz 由执行器负责。
- **动作 7 选 1**：`HOLD_OUTPOST`、`HOLD_BASE`、`TAKE_OUTPOST`、`TAKE_BASE`、
  `RESUPPLY`、`ENGAGE`、`RETREAT`；顺序与 `actions.py` / BT 的 `decision_num` 一致。
- **观测约 59 维**：4 台敌方 ×5 + 结构 4×3 + 控制点 4×3 + 自身 12 + 雷达 2 + 时间 1。
- **雷达是输入通道**：`none` / `synthetic` / `oracle` 三档，用于 A/B/C 消融。
- **版本三件套**：模型 hash、obs 规格 hash、执行器（BT XML + DecisionSwitch）hash，
  每轮训练/评估都记录；改执行器语义必须重评/重训。
- **不喂队友位置**：哨兵按规则只能收雷达数据；观测只用自己的状态 + 雷达 + 公共信息。

## 已知边界

- `Arena1SentryEnv` 是适配器：不传策略时队友/对手是 `StopPolicy`，只适合冒烟；
  正式对局传池（`teammate_pool` / `opponent_pool`）或现成的 `ModulePolicy`。
- Python 执行器与 ROS 行为树必须语义一致；`tests/test_executor_parity.py` 目前是
  TODO，接入真车/仿真时再补固定场景对比。
- 2026 赛季 Arena-1 没有哨兵；本工作区只面向 2027。
