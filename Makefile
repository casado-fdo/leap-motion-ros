ros_master_uri ?= http://127.0.1:11311
ros_ip ?= 127.0.0.1
base_link ?= base_link
pose_smooth_factor ?= 0.95
bones_smooth_factor ?= 0.95
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
		-e ROS_MASTER_URI=${ros_master_uri} \
		-e ROS_IP=${ros_ip} \
		-e MARKERS=${markers} \
		-e RVIZ=${rviz} \
		-e BASE_LINK=${base_link} \
		-e POSE_SMOOTH_FACTOR=${pose_smooth_factor} \
		-e BONES_SMOOTH_FACTOR=${bones_smooth_factor} \
		-v /tmp/.X11-unix:/tmp/.X11-unix \
		-v ./leap_motion_controller:/catkin_ws/src/leap_motion_controller \
		--net host \
		--name lmc lmc:latest \
		bash -c "source devel/setup.bash && roslaunch leap_motion_controller lmc.launch"

debug: .start_if_not_running
	docker exec -it lmc bash

stop:
	docker stop lmc