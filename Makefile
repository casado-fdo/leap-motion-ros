start:
	cpk run -L local --net host -- --privileged \
		-v ./packages:/code/src/leapmotion-docker/packages
rviz:
	cpk run -f -n dev -c bash -M -X --net host -- --privileged 

stop:
	docker stop leapmotion-docker