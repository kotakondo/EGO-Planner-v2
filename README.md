# EGO-Planner-v2
Swarm Playground, the codebase of the paper "[Swarm of micro flying robots in the wild](https://www.science.org/doi/10.1126/scirobotics.abm5954)".

# Installation-Free Usage

Please follow the [tutorial PDF file](swarm-playground/[README]_Brief_Documentation_for_Swarm_Playground.pdf) with corresponding videos ([1](swarm-playground/main_ws/WatchMe_main.mp4), [2](swarm-playground/formation_ws/WatchMe_formation.mp4), [3](swarm-playground/tracking_ws/WatchMe_tracking.mp4), [4](swarm-playground/interlaced_flight_ws/WatchMe_interlaced_flights.mp4)) to run the code.

This work was born out of [MINCO](https://github.com/ZJU-FAST-Lab/GCOPTER).
If you find it interesting, please give both repos stars generously. Thanks.

<img src="images/cover.jpg" alt="drawing" width="400"/>

# Benchmarking
1. Build the docker by `make build` or `make build-no-cache` in the docker folder.
2. Run the docker by `make run` in the docker folder. This will automatically open tmux (run_benchmarking.yaml) and start the benchmarking.
3. For collision check - we need to send the csv file to (a) docker by (`docker cp /home/kkondo/data/hard_forest_obstacle_parameters.csv (here this is docker ps):/home/kota/data`) or (b) the mounted data folder (see Makefile to see where the docker is mounted). You can find the docker ps id by `docker ps`.
3. As specified in Makefile SSD will be mounted so docker data will be saved in the fiolder (e.g. -v /media/kkondo/kota_elements/ego_swwarm_v2:/home/kota/data)

# Benchmarking Setting change
* If you want to change the map, udpate .world file in acl-gazebo/acl_sim/worlds and start_world.launch file in acl-gazebo/acl_sim/launch.

# Benchmarking Errors
* if RVIZ does not start in docker, try `xhost +`
