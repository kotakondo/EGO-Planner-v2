build:
	docker build -t ego_swarm_v2 .

build-no-cache:
	docker build --build-arg DUMMY=`date +%s` -t ego_swarm_v2 .
run:
	docker run  -it \
			--gpus all \
			--env="NVIDIA_DRIVER_CAPABILITIES=all" \
			--privileged \
			-e DISPLAY=${DISPLAY} \
			--network=host \
			--env="DISPLAY" \
			--env="QT_X11_NO_MITSHM=1" \
			-v /media/kkondo/kota_elements/ego_swarm_v2:/home/kota/data \
			--rm ego_swarm_v2
