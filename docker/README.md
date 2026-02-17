# EGO-Swarm v2 Benchmarking Pipeline

Docker-based benchmarking for EGO-Planner v2 against DYNUS environments.
All commands run from inside `docker/`.

## Prerequisites

- Docker with NVIDIA GPU support (`nvidia-docker2`)
- X11 forwarding (`xhost +local:docker` before running with RViz)
- The DYNUS worlds directory at `/home/kkondo/code/dynus_ws/src/dynus/worlds/` (for static benchmarks)
- The DYNUS obstacle CSVs at `/home/kkondo/code/dynus_ws/src/dynus/benchmark_data/static/` (for static collision checking)
- The paper directory at `/home/kkondo/paper_writing/DYNUS_v3/` (for LaTeX table updates)

## 1. Build the Docker Image

```bash
cd docker/
make build
```

Force rebuild (no cache):
```bash
make build-no-cache
```

The image is tagged `ego_swarm_v2`.

---

## 2. Dynamic Benchmark

Runs EGO-Planner with randomly generated dynamic + static obstacles (trefoil-knot trajectories for dynamic obstacles, random boxes for static).

### Environment setup

| Case   | Total Obstacles | Dynamic Ratio | Dynamic | Static |
|--------|-----------------|---------------|---------|--------|
| easy   | 50              | 0.65          | 33      | 17     |
| medium | 100             | 0.65          | 65      | 35     |
| hard   | 200             | 0.65          | 130     | 70     |

Start: `[0, 0, 2]`, Goal: `[105, 0, 2]`, v_max=5 m/s, a_max=20 m/s^2, timeout=100s.

### Run all 3 cases (10 trials each)

```bash
make run-dynamic-benchmark-sweep NUM_TRIALS=10
```

Or run a single case:
```bash
make run-dynamic-benchmark CASE=easy NUM_TRIALS=10
make run-dynamic-benchmark CASE=medium NUM_TRIALS=10
make run-dynamic-benchmark CASE=hard NUM_TRIALS=10
```

Skip the hard case if already collected:
```bash
make run-dynamic-benchmark-sweep SKIP_HARD=1 NUM_TRIALS=10
```

Run with RViz visualization (useful for debugging, 1 trial):
```bash
make run-dynamic-benchmark CASE=easy NUM_TRIALS=1 RVIZ=1
```

### Analyze and update LaTeX table

Analyze all 3 cases and update `dynamic_benchmark.tex`:
```bash
make analyze-dynamic-sweep
```

This overwrites the EGO-Swarm2 row (index 1) within each `\multirow{6}{*}{Easy/Medium/Hard}` block in:
```
/home/kkondo/paper_writing/DYNUS_v3/tables/dynamic_benchmark.tex
```

### Where data is saved

| Host path              | Container path             |
|------------------------|----------------------------|
| `../data_dynamic/`     | `/home/kota/data_dynamic/` |

Each run creates a timestamped directory:
```
data_dynamic/
  easy_benchmark_20260212_143000/
    benchmark_ego_swarm_v2_20260212_143000.csv   # per-trial metrics
    benchmark_ego_swarm_v2_20260212_143000.json  # same, JSON format
    comp_times_trial_0.csv                        # planner computation times
    bags/
      trial_0.bag                                 # rosbag per trial
      trial_1.bag
      ...
  medium_benchmark_20260212_150000/
    ...
  hard_benchmark_20260212_160000/
    ...
```

After analysis, additional files appear in each benchmark dir:
```
benchmark_ego_swarm_v2_postprocessed.csv   # with collision data recomputed from rosbags
benchmark_summary.csv                       # aggregated statistics
```

### Dynamic table layout (`dynamic_benchmark.tex`)

12 columns: `Case | Algorithm(2 cols) | R_succ | T^per_opt | T_trav | L_path | S_jerk | d_min | rho_vel | rho_acc | rho_jerk`

3 case blocks (`\multirow{6}{*}{Easy/Medium/Hard}`), each with:

| Index | Algorithm              |
|-------|------------------------|
| 0     | `\multirow{6}{*}{...}` (case header line) |
| 1     | EGO-Swarm2 (overwritten by `analyze-dynamic-sweep`) |
| 2     | I-MPC v_max=1.5        |
| 3     | I-MPC v_max=3          |
| 4     | I-MPC v_max=5          |
| 5     | FAPP                   |
| 6     | DYNUS                  |

---

## 3. Static Benchmark

Runs EGO-Planner with cylinder obstacles loaded from Gazebo `.world` files (matching DYNUS static environments). Each case is run twice: once with L2 constraint norm and once with L-inf constraint norm (dual-norm benchmarking).

### Environment setup

| Case   | World file              | Cylinders |
|--------|-------------------------|-----------|
| easy   | `easy_forest.world`     | ~166      |
| medium | `medium_forest.world`   | ~633      |
| hard   | `hard_forest.world`     | ~201      |

Same start/goal as dynamic: `[0, 0, 2]` to `[105, 0, 2]`, v_max=5 m/s, timeout=50s.

World files are at: `/home/kkondo/code/dynus_ws/src/dynus/worlds/`

### Run all 3 cases x 2 norms (6 total benchmark runs)

```bash
make run-static-benchmark NUM_TRIALS=10
```

This runs 6 benchmark sets in sequence:
1. `easy` with `USE_LINF_FEAS=0` (L2 norm)
2. `easy` with `USE_LINF_FEAS=1` (L-inf norm)
3. `medium` with `USE_LINF_FEAS=0`
4. `medium` with `USE_LINF_FEAS=1`
5. `hard` with `USE_LINF_FEAS=0`
6. `hard` with `USE_LINF_FEAS=1`

Or run a single case/norm:
```bash
make _run-static-case CASE=easy USE_LINF_FEAS=0 NUM_TRIALS=10   # L2
make _run-static-case CASE=easy USE_LINF_FEAS=1 NUM_TRIALS=10   # L-inf
```

Run with RViz:
```bash
make _run-static-case CASE=easy USE_LINF_FEAS=1 NUM_TRIALS=1 RVIZ=1
```

### Analyze and update LaTeX table

Analyze all 6 benchmark sets and update `static_benchmark.tex`:
```bash
make analyze-static-benchmark
```

This overwrites EGO-Swarm2's dual rows (L2 at index 0, L-inf at index 1) within each `\multirow{6}{*}{Easy/Medium/Hard}` block in:
```
/home/kkondo/paper_writing/DYNUS_v3/tables/static_benchmark.tex
```

### Where data is saved

| Host path             | Container path              |
|-----------------------|-----------------------------|
| `../data_static/`     | `/home/kota/data_static/`   |

Each run creates a timestamped directory with a norm suffix:
```
data_static/
  easy_static_benchmark_20260212_143000_l2/       # L2 norm run
    benchmark_ego_swarm_v2_20260212_143000.csv
    bags/
      trial_0.bag
      ...
  easy_static_benchmark_20260212_144500_linf/     # L-inf norm run
    ...
  medium_static_benchmark_20260212_150000_l2/
    ...
  medium_static_benchmark_20260212_151500_linf/
    ...
  hard_static_benchmark_20260212_160000_l2/
    ...
  hard_static_benchmark_20260212_161500_linf/
    ...
```

### Static table layout (`static_benchmark.tex`)

13 columns: `Env | Algorithm | Constr.Type | Constr.Norm | R_succ | T^total_opt | T^total_replan | T_trav | L_path | S_jerk | rho_vel | rho_acc | rho_jerk`

3 env blocks (`\multirow{6}{*}{Easy/Medium/Hard}`), each with:

| Index | Algorithm    | Constraint       | Norm        |
|-------|-------------|------------------|-------------|
| 0     | `\multirow{2}{*}{EGO-Swarm2}` | `\multirow{2}{*}{Soft}` | L2 (overwritten) |
| 1     | (continuation) | (continuation) | L-inf (overwritten) |
| 2     | `\multirow{2}{*}{SUPER}` | `\multirow{2}{*}{Soft}` | L2 |
| 3     | (continuation) | (continuation) | L-inf |
| 4     | FASTER       | Hard             | L-inf       |
| 5     | DYNUS        | Hard             | L-inf       |

---

## 4. Constraint Violation Methodology

Constraint violations are computed identically to DYNUS for fair comparison:

- **Data source**: `/drone_0_planning/pos_cmd` (PositionCommand) messages, which contain velocity, acceleration, and jerk at each timestep (~100 Hz).
- **Norm**: L-infinity (per-axis max). A violation occurs when `max(|vx|, |vy|, |vz|) > limit + tolerance`.
- **Tolerance**: 1e-3 (matching DYNUS).
- **Rate**: `violating_timesteps / total_timesteps * 100%` (pooled across all successful trials).
- **Limits**: v_max=5.0 m/s, a_max=20.0 m/s^2, j_max=100.0 m/s^3.
- **Success definition**: `goal_reached AND collision_free` (DYNUS methodology). Performance/violation metrics are computed only from successful runs.

Violation checking always uses L-inf norm regardless of the planner's `use_linf_feas` setting. The `--constraint-norm` flag in the analysis script only controls the label in the LaTeX table (L2 vs L-inf), not the violation computation.

---

## 5. Volume Mounts

The Docker containers bind-mount several host directories:

| What                  | Host path (relative to `docker/`)                              | Container path |
|-----------------------|----------------------------------------------------------------|----------------|
| Benchmark data        | `../data_dynamic/` or `../data_static/`                        | `/home/kota/data_dynamic/` or `/home/kota/data_static/` |
| Scripts (live-edit)   | `./` (this directory)                                          | `/home/kota/ego_swarm_v2_ws/src/EGO-Planner-v2/docker/` |
| Launch includes       | `../swarm-playground/.../launch/include/`                      | same path in container |
| Gazebo worlds         | `/home/kkondo/code/dynus_ws/src/dynus/worlds/`                 | `/home/kota/dynus_worlds/` |
| Obstacle CSVs         | `/home/kkondo/code/dynus_ws/src/dynus/benchmark_data/static/`  | `/home/kota/dynus_benchmark_data/` |
| Paper tables          | `/home/kkondo/paper_writing/DYNUS_v3/`                         | `/home/kkondo/paper_writing/DYNUS_v3/` |

Scripts are bind-mounted, so edits to `.py` files take effect immediately without rebuilding the Docker image.

---

## 6. Key Scripts

| Script                     | Purpose |
|----------------------------|---------|
| `run_ego_benchmark.py`     | Main benchmark runner: launches roscore, obstacles, planner, monitors trials, records rosbags |
| `dynamic_forest.py`        | Publishes random dynamic+static obstacles (PointCloud2 + MINCOTraj + MarkerArray) |
| `static_forest.py`         | Parses `.world` XML, publishes cylinder obstacles as PointCloud2 + MarkerArray |
| `analyze_ego_benchmark.py` | Post-processes rosbags (collisions, violations, metrics), computes stats, updates LaTeX tables |

### What `analyze_ego_benchmark.py` does

1. Loads the per-trial CSV from the benchmark run
2. Recomputes computation times from raw `comp_times_trial_*.csv` files (filters -10000 sentinels)
3. Post-processes collisions from rosbags (marker-based for dynamic, CSV-based for static)
4. Recomputes travel time and path length from rosbags
5. Recomputes constraint violations from rosbags (L-inf, per-timestep, matching DYNUS)
6. Computes aggregated statistics (success rate, means, pooled violation rates)
7. Updates the LaTeX table in-place

---

## 7. Debugging

Interactive shell in a container:
```bash
make shell-dynamic    # dynamic env mounts
make shell-static     # static env mounts (includes worlds + obstacle CSVs)
make shell            # basic mounts only
```

Inside the container:
```bash
source /home/kota/ego_swarm_v2_ws/src/EGO-Planner-v2/swarm-playground/main_ws/devel/setup.bash
source /home/kota/mid360_ws/devel/setup.bash
```

---

## 8. Overridable Parameters

All parameters have defaults and can be overridden on the command line:

```bash
# Dynamic benchmark
make run-dynamic-benchmark \
    CASE=easy \
    NUM_TRIALS=20 \
    TIMEOUT=100 \
    NUM_OBSTACLES=50 \
    DYNAMIC_RATIO=0.65 \
    SEED=0 \
    MAX_VEL=5.0 \
    PLANNING_HORIZON=11.25 \
    WEIGHT_TIME=10.0 \
    RVIZ=1

# Static benchmark (single case + norm)
make _run-static-case \
    CASE=easy \
    NUM_TRIALS=10 \
    USE_LINF_FEAS=1 \
    STATIC_TIMEOUT=50 \
    MAX_VEL=5.0 \
    SEED=0 \
    RVIZ=1
```

### Key parameters

| Parameter          | Default | Description |
|--------------------|---------|-------------|
| `NUM_TRIALS`       | 10      | Number of trials per benchmark run |
| `TIMEOUT`          | 100.0   | Dynamic benchmark timeout (seconds) |
| `STATIC_TIMEOUT`   | 50.0    | Static benchmark timeout (seconds) |
| `CASE`             | hard    | Difficulty: easy, medium, hard |
| `NUM_OBSTACLES`    | auto    | Auto-set from CASE (50/100/200) |
| `DYNAMIC_RATIO`    | 0.65    | Fraction of obstacles that are dynamic |
| `MAX_VEL`          | 5.0     | Maximum velocity (m/s) |
| `PLANNING_HORIZON` | 11.25   | Planning horizon (m) |
| `USE_LINF_FEAS`    | 1       | 1=L-inf constraint norm, 0=L2 |
| `RVIZ`             | 1       | 1=show RViz, 0=headless |
| `SKIP_HARD`        | 0       | 1=skip hard case in sweep |
| `SEED`             | 0       | Random seed for obstacle generation |

Run `make help` for a full parameter listing.

---

## 9. Quick Reference

### Full dynamic benchmarking workflow
```bash
cd docker/

# 1. Build (only needed once, or after C++ code changes)
make build

# 2. Run all 3 cases
make run-dynamic-benchmark-sweep NUM_TRIALS=10

# 3. Analyze and update dynamic_benchmark.tex
make analyze-dynamic-sweep
```

### Full static benchmarking workflow
```bash
cd docker/

# 1. Build (only needed once)
make build

# 2. Run all 3 cases x 2 norms (6 runs)
make run-static-benchmark NUM_TRIALS=10

# 3. Analyze and update static_benchmark.tex
make analyze-static-benchmark
```

### Re-analyze existing data (no re-run needed)
```bash
# Dynamic: re-analyze most recent data for all cases
make analyze-dynamic-sweep

# Static: re-analyze most recent data for all cases + norms
make analyze-static-benchmark

# Dynamic: analyze a specific benchmark directory
make analyze-dynamic BENCHMARK_DIR=/home/kota/data_dynamic/easy_benchmark_20260212_143000
```
