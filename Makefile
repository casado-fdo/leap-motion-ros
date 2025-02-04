start:
	@xhost +si:localuser:root >> /dev/null
	cpk run -L local --net host -- --privileged \
		-e DISPLAY \
		-v /tmp/.X11-unix:/tmp/.X11-unix \
		-v /dev:/dev \
		-v ./packages:/code/src/leapmotion-docker/packages
rviz:
	cpk run -f -n dev -c bash -M -X --net host -- --privileged 

stop:
	docker stop leapmotion-docker