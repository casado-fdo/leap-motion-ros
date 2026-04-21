FROM dustynv/ros:noetic-ros-base-l4t-r35.4.1

ARG PROJECT_PATH="/lmc"
RUN mkdir -p ${PROJECT_PATH}
WORKDIR ${PROJECT_PATH}

RUN sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654
RUN apt-get update && apt-get install -y systemd wget python3-pip git build-essential cmake libnss-mdns freeglut3-dev \
    && rm -rf /var/lib/apt/lists/*

RUN wget https://s3.eu-west-1.amazonaws.com/downloads.ultraleap.com/software/tracking-software/6.2.0/tracking-software-raspberry-pi-os-6.2.0.tar.gz --no-check-certificate -O /tmp/tracking-software-raspberry-pi-os-6.2.0.tar.gz
RUN tar -xzf /tmp/tracking-software-raspberry-pi-os-6.2.0.tar.gz -C /tmp/ 
RUN bash /tmp/ultraleap-hand-tracking-service_6.2.0.0-c98d293a-arm64/install_gemini.sh || true
CMD ["libtrack_server", "-g", "daemon off;"]
SHELL ["/bin/bash", "-c"]

RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws
COPY ./leap_motion_controller /catkin_ws/src/leap_motion_controller

RUN python3 -m pip install --upgrade pip
ENV LEAPSDK_INSTALL_LOCATION="/opt/ultraleap/LeapSDK"
RUN git clone --depth 1 https://github.com/ultraleap/leapc-python-bindings.git /opt/leapc-python-bindings
RUN cd /opt/leapc-python-bindings/ \
     && python3 -m pip install -r requirements.txt \
     && python3 -m pip install -e leapc-python-api --use-pep517

RUN apt update
RUN DEBIAN_FRONTEND=noninteractive apt install -y \ 
    python3-rosdep \
    python3-catkin-tools \ 
    python3-rosinstall-generator \
    python3-vcstools \
    python3-vcstool \
    ros-noetic-tf \
    ros-noetic-tf2-ros \
    ros-noetic-sensor-msgs \
    ros-noetic-geometry-msgs \
    ros-noetic-visualization-msgs

RUN pip install pyusb

RUN source /opt/ros/noetic/setup.bash && catkin init && catkin build -DCMAKE_POLICY_VERSION_MINIMUM=3.5 --verbose --workspace /catkin_ws