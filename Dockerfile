FROM osrf/ros:noetic-desktop-full
RUN apt update && apt install -y git wget
ENV DISPLAY=host.docker.internal:0.0
## python-catkin-tools
#### Add User ID and Group ID
ARG UNAME=esvo
ARG UID=1000
ARG GID=1000
RUN groupadd -g $GID -o $UNAME
RUN useradd -m -u $UID -g $GID -o -s /bin/bash $UNAME

# Add User into sudoers, can run sudo command without password
RUN apt update && apt install -y sudo
RUN usermod -aG sudo ${UNAME}
RUN echo "${UNAME} ALL=(ALL) NOPASSWD:ALL" | tee /etc/sudoers.d/${UNAME}

ARG CODE_DIR=/home/${UNAME}

#### ROS setup ####
RUN mkdir -p ${CODE_DIR}/catkin_ws/src && \
    echo "source /opt/ros/noetic/setup.bash" >> ${CODE_DIR}/.bashrc

#### Install dependencies ####
RUN apt-get update && apt-get install -y python3-catkin-tools
RUN apt-get install python3-vcstool

# # install dependencies
# RUN vcs-import < ESVO/dependencies.yaml

#basic environment
RUN apt install -y \
    ca-certificates \
    build-essential \
    git \
    cmake \
    cmake-curses-gui \
    libace-dev \
    libassimp-dev \
    libglew-dev \
    libglfw3-dev \
    libglm-dev \
    libeigen3-dev

# Suggested dependencies for YARP
RUN apt update && apt install -y \
    qtbase5-dev qtdeclarative5-dev qtmultimedia5-dev \
    qml-module-qtquick2 qml-module-qtquick-window2 \
    qml-module-qtmultimedia qml-module-qtquick-dialogs \
    qml-module-qtquick-controls qml-module-qt-labs-folderlistmodel \
    qml-module-qt-labs-settings \
    libqcustomplot-dev \
    libgraphviz-dev \
    libjpeg-dev \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-libav

RUN apt install -y libhdf5-dev


# Add metavision-sdk in sources.list
RUN echo "deb [arch=amd64 trusted=yes] https://apt.prophesee.ai/dists/public/b4b3528d/ubuntu focal sdk" >> /etc/apt/sources.list &&\
    apt update

RUN apt install -y \
    libcanberra-gtk-module \
    mesa-utils \
    ffmpeg \
    libboost-program-options-dev \
    libopencv-dev \
    metavision-sdk
    
# YCM
ARG YCM_VERSION=v0.15.2
RUN cd $CODE_DIR && \
    git clone --depth 1 --branch $YCM_VERSION https://github.com/robotology/ycm.git && \
    cd ycm && \
    mkdir build && cd build && \
    cmake .. && \
    make -j `nproc` install

# Build Open Image Debugger
ARG OID_VERSION=v1.17.30
RUN cd $CODE_DIR && \
    git clone --depth 1 --branch $OID_VERSION https://github.com/OpenImageDebugger/OpenImageDebugger.git && \
    cd OpenImageDebugger && \
    git submodule init && \
    git submodule update && \
    cmake -S . -B build -DCMAKE_INSTALL_PREFIX=/usr/local && \
    cmake --build build --config Release --target install -j 4

# Build Google test
ARG GTEST_VERSION=v1.15.0
RUN cd $CODE_DIR && \
    git clone --depth 1 --branch $GTEST_VERSION https://github.com/google/googletest.git && \
    cd googletest && \
    mkdir build && cd build && \
    cmake .. && \
    make -j `nproc` install

# YARP
ARG YARP_VERSION=v3.8.0
RUN cd $CODE_DIR && \
    git clone --depth 1 --branch $YARP_VERSION https://github.com/robotology/yarp.git &&\
    cd yarp &&\
    mkdir build && cd build &&\
    cmake .. &&\
    make -j `nproc` install

EXPOSE 10000/tcp 10000/udp
RUN yarp check


# event-driven
ARG ED_VERSION=main
RUN cd $CODE_DIR &&\
    git clone --depth 1 --branch $ED_VERSION https://github.com/robotology/event-driven.git &&\
    cd event-driven &&\
    mkdir build && cd build &&\
    cmake .. &&\
    make -j `nproc` install





# #### Install dependencies ####
# RUN apt-get update && apt-get install -y python-catkin-tools python3-vcstool
# # install dependencies
# RUN vcs-import < ESVO/dependencies.yaml

# RUN cd /app/catkin_ws/src && \
#     git clone https://github.com/jbeder/yaml-cpp.git && \
#     cd yaml-cpp && \
#     mkdir build && cd build && cmake -DYAML_BUILD_SHARED_LIBS=ON .. && \
#     make -j

# build libcaer library
RUN mkdir ${CODE_DIR}/src && cd ${CODE_DIR}/src && \
    git clone https://gitlab.com/inivation/dv/libcaer.git && cd libcaer && \
    apt-get install -y build-essential cmake pkg-config libusb-1.0-0-dev && \
    cmake -DCMAKE_INSTALL_PREFIX=/usr . && \
    make && make install
# build ceres
RUN apt-get install -y libgoogle-glog-dev libgflags-dev libatlas-base-dev && \
    cd ${CODE_DIR}/src && wget http://ceres-solver.org/ceres-solver-2.1.0.tar.gz && \
    tar zxf ceres-solver-2.1.0.tar.gz && \
    mkdir -p ${CODE_DIR}/src/ceres-solver-2.1.0/build && cd ${CODE_DIR}/src/ceres-solver-2.1.0/build && \
    cmake .. && make && make install

#### Copy source code ####
COPY ./ ${CODE_DIR}/catkin_ws/src

RUN chown -R $UNAME:$UNAME ${CODE_DIR}/catkin_ws

USER $UNAME
WORKDIR ${CODE_DIR}

