FROM personalroboticsimperial/ubuntu:22.04-amd64


ARG PROJECT_PATH="/lmc"
RUN mkdir -p ${PROJECT_PATH}
WORKDIR ${PROJECT_PATH}

RUN apt update && apt install -y gnupg2 && rm -rf /var/lib/apt/lists/*
RUN wget -qO - https://repo.ultraleap.com/keys/apt/gpg | gpg --dearmor | tee /etc/apt/trusted.gpg.d/ultraleap.gpg
RUN echo 'deb [arch=amd64] https://repo.ultraleap.com/apt stable main' | tee /etc/apt/sources.list.d/ultraleap.list

RUN apt update && apt install -y freeglut3-dev \
    git \
    libnss-mdns \
    ultraleap-hand-tracking

RUN wget -qO Miniforge3.sh https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-$(uname)-$(uname -m).sh \
    && bash Miniforge3.sh -b -p /opt/miniforge3 \
    && rm Miniforge3.sh

SHELL ["/bin/bash", "-c"]

RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws
COPY ./leap_motion_controller /catkin_ws/src/leap_motion_controller

RUN . /opt/miniforge3/etc/profile.d/conda.sh \
    && . /opt/miniforge3/etc/profile.d/mamba.sh \
    && mamba create -n ros \
    && mamba activate ros \
    && pip install -U pip \
    && conda config --env --add channels robostack-staging \
    && mamba install -y ros-noetic-desktop-full \
    && git clone --depth 1 https://github.com/ultraleap/leapc-python-bindings.git /opt/leapc-python-bindings \
    && cd /opt/leapc-python-bindings/ \
    && mkdir src \
    && pip install -r requirements.txt \
    && python -m build leapc-cffi \
    && pip install leapc-cffi/dist/leapc_cffi-0.0.1.tar.gz \
    && pip install -e leapc-python-api \
    && pip install empy==3.* \
    && catkin init && catkin build --workspace /catkin_ws
    
ENV GAZEBO_RESOURCE_PATH=""
ENV GAZEBO_PLUGIN_PATH=""
ENV GAZEBO_MODEL_PATH=""
ENV LD_LIBRARY_PATH=""
ENV CONDA_BUILD=""
ENV ZSH_VERSION=""
ENV ROS_DISTRO="noetic"
ENV target_platform="amd64"
ENV build_platform="amd64"
ENV ROS_MASTER_URI="http://localhost:11311"

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
