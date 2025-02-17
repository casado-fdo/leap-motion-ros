BASE_IP ?= 127.0.0.1

.build:
	docker build -t lmc:latest -f Dockerfile .

start:
	@xhost +si:localuser:root >> /dev/null
	docker run -it --rm --privileged \
		-e DISPLAY \
		-e ROS_MASTER_URI="http://${BASE_IP}:11311" \
		-e ROS_IP=${BASE_IP} \
		-v /tmp/.X11-unix:/tmp/.X11-unix \
		-v ./leap_motion_controller:/catkin_ws/src/leap_motion_controller \
		--net host \
		--name lmc lmc:latest \
		bash -c "leapctl eula -y && leapd & roslaunch leap_motion_controller lmc.launch"

stop:
	docker stop lmc