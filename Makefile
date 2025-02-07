.build:
	docker build -t lmc:latest -f Dockerfile .

start:
	@xhost +si:localuser:root >> /dev/null
	docker run -it --rm --privileged \
		-e DISPLAY \
		-v /tmp/.X11-unix:/tmp/.X11-unix \
		--net host \
		-v /dev:/dev \
		--name lmc lmc:latest \
		bash -c "leapctl eula -y && leapd & roslaunch leap_motion_controller lmc.launch"

stop:
	docker stop lmc