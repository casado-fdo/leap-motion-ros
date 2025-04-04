base_ip ?= 127.0.0.1
base_link ?= base_link
markers ?= true
rviz ?= false

.build:
	docker build -t lmc:latest -f Dockerfile .

.start_if_not_running:
	@if ! docker ps -a | grep -w lmc; then $(MAKE) start; fi

start:
	@xhost +si:localuser:root >> /dev/null
	docker run -it --rm --privileged \
		-e DISPLAY \
		-e ROS_MASTER_URI="http://${base_ip}:11311" \
		-e ROS_IP=${base_ip} \
		-e MARKERS=${markers} \
		-e RVIZ=${rviz} \
		-e BASE_LINK=${base_link} \
		-v /tmp/.X11-unix:/tmp/.X11-unix \
		-v ./leap_motion_controller:/catkin_ws/src/leap_motion_controller \
		--net host \
		--name lmc lmc:latest \
		bash -c "leapctl eula -y && leapd & roslaunch leap_motion_controller lmc.launch"

debug: .start_if_not_running
	docker exec -it lmc bash

stop:
	docker stop lmc