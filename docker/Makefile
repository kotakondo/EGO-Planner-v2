# Makefile for EGO-Swarm v2 Docker benchmarking pipeline

.PHONY: build build-no-cache run-sim run-benchmark analyze check-collisions \
        compute-stats analyze-all shell stop clean help \
        run-dynamic-sim run-dynamic-benchmark analyze-dynamic \
        check-collisions-dynamic compute-stats-dynamic analyze-all-dynamic \
        shell-dynamic run-dynamic-benchmark-sweep analyze-dynamic-sweep \
        run-static-benchmark _run-static-case analyze-static-benchmark \
        _analyze-static-case shell-static

# ── Image name ────────────────────────────────────────────────────────────────
IMAGE_NAME := ego_swarm_v2

# ── Benchmark parameters (override on command line) ───────────────────────────
NUM_TRIALS     ?= 10
MAX_DURATION   ?= 300.0
GOAL           ?= 305.0,0.0,3.0
GOAL_THRESHOLD ?= 0.5

# ── Analysis parameters ──────────────────────────────────────────────────────
V_MAX        ?= 5.0
A_MAX        ?= 20.0
J_MAX        ?= 100.0
DRONE_RADIUS ?= 0.1
FOREST_CSV   ?= hard_forest_obstacle_parameters.csv

# ── Dynamic environment parameters ─────────────────────────────────────────
CASE           ?= hard
DYNAMIC_RATIO  ?= 0.65
SEED           ?= 0
TIMEOUT        ?= 100.0
RVIZ           ?= 1
WEIGHT_TIME    ?= 10.0
MAX_VEL            ?= 5.0
PLANNING_HORIZON   ?= 11.25
ALG_NAME       ?= \\multicolumn{2}{c}{EGO-Swarm2}
MULTIROW_GROUP ?=
MULTIROW_SIZE  ?= 6
MULTIROW_INDEX ?= 1
CASE_VALUE     ?=
SKIP_HARD      ?= 0
USE_LINF_FEAS  ?= 1
CONSTRAINT_NORM ?= linf

# Map CASE to default NUM_OBSTACLES (overridable on command line)
ifeq ($(CASE),easy)
  NUM_OBSTACLES ?= 50
else ifeq ($(CASE),medium)
  NUM_OBSTACLES ?= 100
else
  NUM_OBSTACLES ?= 200
endif

# ── Host data directory (mounted into container at /home/kota/data) ──────────
DATA_DIR ?= ../data
DATA_DIR_DYNAMIC ?= ../data_dynamic
DATA_DIR_STATIC ?= ../data_static

# ── Static benchmark: Gazebo world files & obstacle CSVs ────────────────────
WORLDS_DIR_HOST     ?= /home/kkondo/code/dynus_ws/src/dynus/worlds
BENCHMARK_DATA_HOST ?= /home/kkondo/code/dynus_ws/src/dynus/benchmark_data/static
STATIC_TIMEOUT      ?= 50.0

# ── Common docker run flags ──────────────────────────────────────────────────
DOCKER_RUN_FLAGS := \
	--gpus all \
	--env="NVIDIA_DRIVER_CAPABILITIES=all" \
	--privileged \
	-e DISPLAY=$(DISPLAY) \
	--network=host \
	--env="QT_X11_NO_MITSHM=1" \
	--env="DISABLE_ROS1_EOL_WARNINGS=1" \
	--volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
	-v $(abspath $(DATA_DIR)):/home/kota/data \
	--rm

# ── Source command used inside the container ─────────────────────────────────
ENV_SOURCE := source /home/kota/ego_swarm_v2_ws/src/EGO-Planner-v2/swarm-playground/main_ws/devel/setup.bash && \
              source /home/kota/mid360_ws/devel/setup.bash

# ── Scripts path inside the container ────────────────────────────────────────
SCRIPTS := /home/kota/ego_swarm_v2_ws/src/EGO-Planner-v2/docker

# ── Targets ──────────────────────────────────────────────────────────────────

## Build the Docker image
build:
	docker build -t $(IMAGE_NAME) .

## Force rebuild without cache
build-no-cache:
	docker build --build-arg DUMMY=`date +%s` --no-cache -t $(IMAGE_NAME) .

## Interactive simulation via tmuxp
run-sim:
	docker run -it $(DOCKER_RUN_FLAGS) $(IMAGE_NAME) \
		/bin/bash -c "$(ENV_SOURCE) && tmuxp load /ego_swarm_v2_sim.yml"

## Run N benchmark trials
run-benchmark:
	@echo "EGO-Swarm v2 Benchmark"
	@echo "  NUM_TRIALS     = $(NUM_TRIALS)"
	@echo "  MAX_DURATION   = $(MAX_DURATION)"
	@echo "  GOAL           = $(GOAL)"
	@echo "  GOAL_THRESHOLD = $(GOAL_THRESHOLD)"
	@echo ""
	@echo "Data will be saved to: $(abspath $(DATA_DIR))/"
	@mkdir -p $(DATA_DIR)
	docker run -it $(DOCKER_RUN_FLAGS) $(IMAGE_NAME) \
		/bin/bash -c "\
			$(ENV_SOURCE) && \
			roscore & \
			sleep 3 && \
			python3 $(SCRIPTS)/benchmarking.py \
				$(NUM_TRIALS) $(MAX_DURATION) $(GOAL) $(GOAL_THRESHOLD)"

## Analyze bag files (travel time, smoothness, violations)
analyze:
	@echo "Analyzing bag files in $(abspath $(DATA_DIR))/ ..."
	docker run -it $(DOCKER_RUN_FLAGS) $(IMAGE_NAME) \
		/bin/bash -c "\
			$(ENV_SOURCE) && \
			cd $(SCRIPTS) && \
			python3 analyze_bag.py /home/kota/data $(V_MAX) $(A_MAX) $(J_MAX)"

## Check collisions against forest obstacles
check-collisions:
	@echo "Checking collisions (drone_radius=$(DRONE_RADIUS), forest=$(FOREST_CSV)) ..."
	docker run -it $(DOCKER_RUN_FLAGS) $(IMAGE_NAME) \
		/bin/bash -c "\
			$(ENV_SOURCE) && \
			cd $(SCRIPTS) && \
			python3 collision_checker.py \
				/home/kota/data/$(FOREST_CSV) /home/kota/data \
				$(DRONE_RADIUS) /drone_0_planning/pos_cmd"

## Compute average computation times
compute-stats:
	@echo "Computing average computation times ..."
	docker run -it $(DOCKER_RUN_FLAGS) $(IMAGE_NAME) \
		/bin/bash -c "\
			cd $(SCRIPTS) && \
			python3 compute_average_computation.py /home/kota/data"

## Run all analysis targets
analyze-all: analyze check-collisions compute-stats

# ── Dynamic environment docker run flags ───────────────────────────────────
# Bind-mount host docker/ dir so script edits are instantly visible (no rebuild).
# Also bind-mount the launch/include dir so rviz config changes take effect.
LAUNCH_DIR := /home/kota/ego_swarm_v2_ws/src/EGO-Planner-v2/swarm-playground/main_ws/src/planner/plan_manage/launch
LAUNCH_INCLUDE_HOST := ../swarm-playground/main_ws/src/planner/plan_manage/launch/include

PAPER_DIR ?= /home/kkondo/paper_writing/DYNUS_v3

DOCKER_RUN_FLAGS_DYNAMIC := \
	--gpus all \
	--env="NVIDIA_DRIVER_CAPABILITIES=all" \
	--privileged \
	-e DISPLAY=$(DISPLAY) \
	--network=host \
	--env="QT_X11_NO_MITSHM=1" \
	--env="DISABLE_ROS1_EOL_WARNINGS=1" \
	--volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
	-v $(abspath $(DATA_DIR_DYNAMIC)):/home/kota/data_dynamic \
	-v $(abspath .):/home/kota/ego_swarm_v2_ws/src/EGO-Planner-v2/docker \
	-v $(abspath $(LAUNCH_INCLUDE_HOST)):$(LAUNCH_DIR)/include \
	-v $(abspath $(PAPER_DIR)):/home/kkondo/paper_writing/DYNUS_v3 \
	--rm

# ── Dynamic environment targets ───────────────────────────────────────────

## Interactive dynamic simulation via tmuxp
run-dynamic-sim:
	@mkdir -p $(DATA_DIR_DYNAMIC)
	docker run -it $(DOCKER_RUN_FLAGS_DYNAMIC) $(IMAGE_NAME) \
		/bin/bash -c "\
			cp $(SCRIPTS)/single_drone_waypoints_dynamic.launch $(LAUNCH_DIR)/ && \
			cp $(SCRIPTS)/ego_swarm_v2_dynamic_sim.yml / && \
			$(ENV_SOURCE) && tmuxp load /ego_swarm_v2_dynamic_sim.yml"

## Run N benchmark trials in dynamic environment
run-dynamic-benchmark:
	@echo "EGO-Swarm v2 Dynamic Benchmark"
	@echo "  CASE            = $(CASE)"
	@echo "  NUM_TRIALS      = $(NUM_TRIALS)"
	@echo "  TIMEOUT         = $(TIMEOUT)"
	@echo "  NUM_OBSTACLES   = $(NUM_OBSTACLES)"
	@echo "  DYNAMIC_RATIO   = $(DYNAMIC_RATIO)"
	@echo "  SEED            = $(SEED)"
	@echo ""
	@echo "Data will be saved to: $(abspath $(DATA_DIR_DYNAMIC))/"
	@mkdir -p $(DATA_DIR_DYNAMIC)
	docker run -it $(DOCKER_RUN_FLAGS_DYNAMIC) $(IMAGE_NAME) \
		/bin/bash -c "\
			cp $(SCRIPTS)/single_drone_waypoints_dynamic.launch $(LAUNCH_DIR)/ && \
			$(ENV_SOURCE) && \
			python3 $(SCRIPTS)/run_ego_benchmark.py \
				--num-trials $(NUM_TRIALS) \
				--timeout $(TIMEOUT) \
				--num-obstacles $(NUM_OBSTACLES) \
				--dynamic-ratio $(DYNAMIC_RATIO) \
				--seed-start $(SEED) \
				--output-dir /home/kota/data_dynamic \
				--weight-time $(WEIGHT_TIME) \
				--max-vel $(MAX_VEL) \
				--planning-horizon $(PLANNING_HORIZON) \
				--case-name $(CASE) \
				$(if $(filter-out 0,$(RVIZ)),--rviz,)"

## Analyze latest dynamic benchmark (collisions + stats + LaTeX table)
## Override BENCHMARK_DIR to analyze a specific run, e.g.:
##   make analyze-dynamic BENCHMARK_DIR=/home/kota/data_dynamic/benchmark_20260207_120000
## For multirow sweep rows, set MULTIROW_GROUP, MULTIROW_INDEX, CASE_VALUE.
analyze-dynamic:
	@echo "Analyzing dynamic benchmark data ..."
	docker run -it $(DOCKER_RUN_FLAGS_DYNAMIC) $(IMAGE_NAME) \
		/bin/bash -c "\
			pip install -q pandas && \
			$(ENV_SOURCE) && \
			cd $(SCRIPTS) && \
			BDIR=$${BENCHMARK_DIR:-\$$(ls -dt /home/kota/data_dynamic/*_benchmark_* 2>/dev/null | head -1)} && \
			if [ -z \"\$$BDIR\" ]; then echo 'ERROR: No benchmark directory found in /home/kota/data_dynamic/'; exit 1; fi && \
			echo \"Analyzing: \$$BDIR\" && \
			python3 analyze_ego_benchmark.py --data-dir \$$BDIR --algorithm-name '$(ALG_NAME)' $(if $(MULTIROW_GROUP),--multirow-group '$(MULTIROW_GROUP)' --multirow-size $(MULTIROW_SIZE) --multirow-index $(MULTIROW_INDEX) --case-value '$(CASE_VALUE)',)"

## Run all dynamic analysis (alias)
analyze-all-dynamic: analyze-dynamic

## Run benchmark sweep over difficulty cases (easy=50, medium=100, hard=200 obstacles)
## Set SKIP_HARD=1 to skip the hard case (e.g. if already collected)
run-dynamic-benchmark-sweep:
	$(MAKE) run-dynamic-benchmark CASE=easy NUM_OBSTACLES=50
	$(MAKE) run-dynamic-benchmark CASE=medium NUM_OBSTACLES=100
ifneq ($(SKIP_HARD),1)
	$(MAKE) run-dynamic-benchmark CASE=hard NUM_OBSTACLES=200
endif

## Analyze all case sweep results and update LaTeX table
## Overwrites the EGO-Swarm2 row (index 1) within each case block:
##   \multirow{6}{*}{Easy}                                     (index 0)
##                         & EGO-Swarm2            & ...       (index 1) ← overwritten
##                         & I-MPC      & vmax=1.5 & ...       (index 2)
##                         & I-MPC      & vmax=3   & ...       (index 3)
##                         & I-MPC      & vmax=5   & ...       (index 4)
##                         & FAPP                  & ...       (index 5)
##                         & DYNUS                 & ...       (index 6)
analyze-dynamic-sweep:
	for case_name in easy medium hard; do \
		case_cap=$$(echo $$case_name | sed 's/./\U&/'); \
		BDIR=$$(ls -dt $(abspath $(DATA_DIR_DYNAMIC))/$${case_name}_benchmark_* 2>/dev/null | head -1); \
		if [ -z "$$BDIR" ]; then echo "WARNING: No benchmark dir for case=$$case_name, skipping"; continue; fi; \
		BDIR_NAME=$$(basename "$$BDIR"); \
		$(MAKE) analyze-dynamic \
			BENCHMARK_DIR=/home/kota/data_dynamic/$$BDIR_NAME \
			ALG_NAME='\\multicolumn{2}{c}{EGO-Swarm2}' \
			MULTIROW_GROUP=$$case_cap \
			MULTIROW_SIZE=6 \
			MULTIROW_INDEX=1 \
			CASE_VALUE=$$case_name \
		|| true; \
	done

# ── Static environment docker run flags ───────────────────────────────────
# Same as dynamic but mounts data_static and the dynus worlds directory.
DOCKER_RUN_FLAGS_STATIC := \
	--gpus all \
	--env="NVIDIA_DRIVER_CAPABILITIES=all" \
	--privileged \
	-e DISPLAY=$(DISPLAY) \
	--network=host \
	--env="QT_X11_NO_MITSHM=1" \
	--env="DISABLE_ROS1_EOL_WARNINGS=1" \
	--volume=/tmp/.X11-unix:/tmp/.X11-unix:rw \
	-v $(abspath $(DATA_DIR_STATIC)):/home/kota/data_static \
	-v $(abspath .):/home/kota/ego_swarm_v2_ws/src/EGO-Planner-v2/docker \
	-v $(abspath $(LAUNCH_INCLUDE_HOST)):$(LAUNCH_DIR)/include \
	-v $(WORLDS_DIR_HOST):/home/kota/dynus_worlds \
	-v $(BENCHMARK_DATA_HOST):/home/kota/dynus_benchmark_data:ro \
	-v $(abspath $(PAPER_DIR)):/home/kkondo/paper_writing/DYNUS_v3 \
	--rm

# ── Static environment targets ──────────────────────────────────────────

## Run static benchmark (10 trials each in easy, medium, hard forests, both L2 and L-inf)
run-static-benchmark:
	$(MAKE) _run-static-case CASE=easy USE_LINF_FEAS=0
	$(MAKE) _run-static-case CASE=easy USE_LINF_FEAS=1
	$(MAKE) _run-static-case CASE=medium USE_LINF_FEAS=0
	$(MAKE) _run-static-case CASE=medium USE_LINF_FEAS=1
	$(MAKE) _run-static-case CASE=hard USE_LINF_FEAS=0
	$(MAKE) _run-static-case CASE=hard USE_LINF_FEAS=1

_run-static-case:
	@echo "EGO-Swarm v2 Static Benchmark"
	@echo "  CASE           = $(CASE)"
	@echo "  NUM_TRIALS     = $(NUM_TRIALS)"
	@echo "  TIMEOUT        = $(STATIC_TIMEOUT)"
	@echo "  USE_LINF_FEAS  = $(USE_LINF_FEAS)"
	@echo ""
	@echo "Data will be saved to: $(abspath $(DATA_DIR_STATIC))/"
	@mkdir -p $(DATA_DIR_STATIC)
	docker run -it $(DOCKER_RUN_FLAGS_STATIC) $(IMAGE_NAME) \
		/bin/bash -c "\
			cp $(SCRIPTS)/single_drone_waypoints_dynamic.launch $(LAUNCH_DIR)/ && \
			$(ENV_SOURCE) && \
			python3 $(SCRIPTS)/run_ego_benchmark.py \
				--num-trials $(NUM_TRIALS) \
				--timeout $(STATIC_TIMEOUT) \
				--num-obstacles 0 \
				--dynamic-ratio 0.0 \
				--seed-start $(SEED) \
				--output-dir /home/kota/data_static \
				--max-vel $(MAX_VEL) \
				--planning-horizon $(PLANNING_HORIZON) \
				--case-name $(CASE) \
				--static-world /home/kota/dynus_worlds/$(CASE)_forest.world \
				--use-linf-feas $(USE_LINF_FEAS) \
				$(if $(filter-out 0,$(RVIZ)),--rviz,)"

## Analyze static benchmark results and update static_benchmark.tex
## Overwrites EGO-Swarm2 rows (indices 0-1) within each Env block:
##   \multirow{6}{*}{Easy} & \multirow{2}{*}{EGO-Swarm2} & Soft & $L_2$      & ...  (index 0)
##                          &                              & Soft & $L_\infty$ & ...  (index 1)
##                          & \multirow{2}{*}{SUPER}       & Soft & $L_2$      & ...  (index 2)
##                          &                              & Soft & $L_\infty$ & ...  (index 3)
##                          & FASTER                       & Hard & $L_\infty$ & ...  (index 4)
##                          & DYNUS                        & Hard & $L_\infty$ & ...  (index 5)
analyze-static-benchmark:
	for case_name in easy medium hard; do \
		case_cap=$$(echo $$case_name | sed 's/./\U&/'); \
		for norm_info in "l2:0:\\\\multirow{2}{*}{EGO-Swarm2}" "linf:1:"; do \
			norm=$$(echo $$norm_info | cut -d: -f1); \
			idx=$$(echo $$norm_info | cut -d: -f2); \
			alg=$$(echo $$norm_info | cut -d: -f3); \
			BDIR=$$(ls -dt $(abspath $(DATA_DIR_STATIC))/$${case_name}_static_benchmark_*_$${norm} 2>/dev/null | head -1); \
			if [ -z "$$BDIR" ]; then echo "WARNING: No benchmark dir for case=$$case_name norm=$$norm, skipping"; continue; fi; \
			BDIR_NAME=$$(basename "$$BDIR"); \
			$(MAKE) _analyze-static-case \
				BENCHMARK_DIR=/home/kota/data_static/$$BDIR_NAME \
				MULTIROW_GROUP=$$case_cap \
				MULTIROW_SIZE=6 \
				MULTIROW_INDEX=$$idx \
				ALG_NAME="$$alg" \
				CONSTRAINT_NORM=$$norm \
				OBSTACLE_CSV=/home/kota/dynus_benchmark_data/$${case_name}_forest_obstacle_parameters.csv \
			|| true; \
		done; \
	done

OBSTACLE_CSV ?=

_analyze-static-case:
	docker run -it $(DOCKER_RUN_FLAGS_STATIC) $(IMAGE_NAME) \
		/bin/bash -c "\
			pip install -q pandas && \
			$(ENV_SOURCE) && \
			cd $(SCRIPTS) && \
			echo \"Analyzing: $(BENCHMARK_DIR)\" && \
			python3 analyze_ego_benchmark.py --data-dir $(BENCHMARK_DIR) \
				--algorithm-name '$(ALG_NAME)' \
				--dynus-table-path /home/kkondo/paper_writing/DYNUS_v3/tables/static_benchmark.tex \
				--table-format static \
				--multirow-group '$(MULTIROW_GROUP)' \
				--multirow-size $(MULTIROW_SIZE) \
				--multirow-index $(MULTIROW_INDEX) \
				--constraint-norm $(CONSTRAINT_NORM) \
				$(if $(OBSTACLE_CSV),--obstacle-csv '$(OBSTACLE_CSV)',)"

## Interactive bash shell in the static container
shell-static:
	@mkdir -p $(DATA_DIR_STATIC)
	docker run -it $(DOCKER_RUN_FLAGS_STATIC) $(IMAGE_NAME) /bin/bash

## Interactive bash shell in the dynamic container
shell-dynamic:
	@mkdir -p $(DATA_DIR_DYNAMIC)
	docker run -it $(DOCKER_RUN_FLAGS_DYNAMIC) $(IMAGE_NAME) /bin/bash

## Interactive bash shell in the container
shell:
	docker run -it $(DOCKER_RUN_FLAGS) $(IMAGE_NAME) /bin/bash

## Stop running containers for this image
stop:
	@echo "Stopping all $(IMAGE_NAME) containers ..."
	-docker ps -q --filter ancestor=$(IMAGE_NAME) | xargs -r docker stop

## Remove stopped containers
clean:
	docker container prune -f

## Print usage information
help:
	@echo "EGO-Swarm v2 Docker Makefile"
	@echo ""
	@echo "Targets (static forest):"
	@echo "  build            Build the Docker image"
	@echo "  build-no-cache   Force rebuild without cache"
	@echo "  run-sim          Interactive simulation via tmuxp"
	@echo "  run-benchmark    Run N benchmark trials"
	@echo "  analyze          Analyze bag files (travel time, smoothness, violations)"
	@echo "  check-collisions Check collisions against forest obstacles"
	@echo "  compute-stats    Compute average computation times"
	@echo "  analyze-all      Run analyze + check-collisions + compute-stats"
	@echo ""
	@echo "Targets (dynamic environment):"
	@echo "  run-dynamic-sim              Interactive dynamic simulation via tmuxp"
	@echo "  run-dynamic-benchmark        Run N trials in dynamic environment"
	@echo "  run-dynamic-benchmark-sweep  Sweep cases: easy (50), medium (100), hard (200)"
	@echo "  analyze-dynamic              Analyze latest dynamic benchmark (collisions + stats + LaTeX)"
	@echo "  analyze-dynamic-sweep        Analyze all case sweep results (multirow)"
	@echo "  analyze-all-dynamic          Alias for analyze-dynamic"
	@echo "  shell-dynamic                Interactive bash in dynamic container"
	@echo ""
	@echo "Targets (static environment):"
	@echo "  run-static-benchmark         Run 10 trials each in easy, medium, hard forests"
	@echo "  analyze-static-benchmark     Analyze all static results + update static_benchmark.tex"
	@echo "  shell-static                 Interactive bash in static container"
	@echo ""
	@echo "General:"
	@echo "  shell            Interactive bash in the container"
	@echo "  stop             Stop running containers"
	@echo "  clean            Remove stopped containers"
	@echo "  help             Print this message"
	@echo ""
	@echo "Benchmark parameters (defaults):"
	@echo "  NUM_TRIALS=$(NUM_TRIALS)  MAX_DURATION=$(MAX_DURATION)  GOAL=$(GOAL)  GOAL_THRESHOLD=$(GOAL_THRESHOLD)"
	@echo ""
	@echo "Dynamic environment parameters (defaults):"
	@echo "  CASE=$(CASE)  NUM_OBSTACLES=$(NUM_OBSTACLES)  DYNAMIC_RATIO=$(DYNAMIC_RATIO)  SEED=$(SEED)  TIMEOUT=$(TIMEOUT)"
	@echo "  RVIZ=$(RVIZ)  SKIP_HARD=$(SKIP_HARD)  ALG_NAME=$(ALG_NAME)"
	@echo ""
	@echo "Analysis parameters (defaults):"
	@echo "  V_MAX=$(V_MAX)  A_MAX=$(A_MAX)  J_MAX=$(J_MAX)"
	@echo "  DRONE_RADIUS=$(DRONE_RADIUS)  FOREST_CSV=$(FOREST_CSV)"
	@echo ""
	@echo "Static environment parameters (defaults):"
	@echo "  WORLDS_DIR_HOST=$(WORLDS_DIR_HOST)  STATIC_TIMEOUT=$(STATIC_TIMEOUT)"
	@echo ""
	@echo "Data directories:"
	@echo "  DATA_DIR=$(DATA_DIR) (mounted at /home/kota/data)"
	@echo "  DATA_DIR_DYNAMIC=$(DATA_DIR_DYNAMIC) (mounted at /home/kota/data_dynamic)"
	@echo "  DATA_DIR_STATIC=$(DATA_DIR_STATIC) (mounted at /home/kota/data_static)"
	@echo ""
	@echo "Examples:"
	@echo "  make build"
	@echo "  make run-benchmark NUM_TRIALS=20 MAX_DURATION=600"
	@echo "  make analyze-all"
	@echo "  make run-dynamic-benchmark CASE=easy NUM_TRIALS=20"
	@echo "  make run-dynamic-benchmark CASE=medium NUM_TRIALS=20"
	@echo "  make run-dynamic-benchmark NUM_TRIALS=1 RVIZ=1    # with RViz visualization"
	@echo "  make run-dynamic-benchmark-sweep NUM_TRIALS=20    # sweep easy+medium+hard"
	@echo "  make run-dynamic-benchmark-sweep SKIP_HARD=1 NUM_TRIALS=20  # only easy+medium"
	@echo "  make analyze-dynamic"
	@echo "  make analyze-dynamic-sweep                        # analyze all case results (multirow)"
	@echo "  make run-static-benchmark                         # sweep easy+medium+hard forests"
	@echo "  make run-static-benchmark NUM_TRIALS=5            # fewer trials"
	@echo "  make analyze-static-benchmark                     # analyze all static results"
	@echo "  make shell"
