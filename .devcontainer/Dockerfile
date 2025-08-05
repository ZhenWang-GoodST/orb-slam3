FROM osrf/ros:noetic-desktop-full
# FROM ros:noetic-perception
#FROM ros:fast_livo2
#build: docker build --tag ros:noetic -f ./Dockerfile /home/cvrsg/docker/build_dir/orb_slam --no-cache

ENV CERES_VERSION="1.12.0"
ENV CATKIN_WS=/root/catkin_ws

COPY ./ros1-latest.list  /etc/apt/sources.list.d
COPY ./ros-archive-keyring.gpg /usr/share/keyrings/
# 清除Docker缓存
RUN echo "Clear cache test"

RUN   sudo sed -i "s@http://.*archive.ubuntu.com@http://repo.huaweicloud.com@g" /etc/apt/sources.list && \
      sudo sed -i "s@http://.*security.ubuntu.com@http://repo.huaweicloud.com@g" /etc/apt/sources.list && \
      ls / && \
      apt-get update && apt-get install -y \
      gdb \
      git \
      cmake \
      libatlas-base-dev \
      libgoogle-glog-dev \
      libsuitesparse-dev \
      python3-catkin-tools \
      libpcl-dev \
      libeigen3-dev \
      # libopencv-dev \
      libcanberra-gtk-module \
      libcanberra-gtk3-module \
      libceres-dev \
      libgsl-dev \
      bash-completion \
      libarpack* && \
    #   apt-get remove ros-noetic-cv-bridge -y && \
      # ros-${ROS_DISTRO}-cv-bridge \
      # ros-${ROS_DISTRO}-image-transport \
      # ros-${ROS_DISTRO}-message-filters \
      # ros-${ROS_DISTRO}-tf && \
      # rm -rf /var/lib/apt/lists/* && \
      #Build and install Ceres
      mkdir -p /github/json && mkdir -p  /github/Pangolin && mkdir -p  /github/glog
      #Build and install json
      # cd /home/cvrsg/github/json && mkdir build && cd build && cmake .. && make install


# Copy 
COPY ./json  /github/json
COPY ./Pangolin  /github/Pangolin
COPY ./glog  /github/glog

      # set up thread number for building
RUN   ls / && if [ "x$(nproc)" = "x1" ] ; then export USE_PROC=1 ; \
      else export USE_PROC=$(($(nproc)/2)) ; fi && \
      cd /github/json && \
      mkdir build && cd build && \
      cmake .. && \
      make -j32 && make install && \
      cd /github/Pangolin && \
      mkdir build && cd build && \
      cmake .. && \
      make -j32 && make install && \
      cd /github/glog && \
      mkdir build && cd build && \
      cmake .. && \
      make -j32 && make install && \
      rm -rf /github
      # mkdir -p $CATKIN_WS/src/VINS-Mono/

WORKDIR /home/cvrsg/gitee/plvins_catkinws

ENTRYPOINT ["/ros_entrypoint.sh"]

LABEL Name=ros:pl_vins Version=0.0.1
# # Copy VINS-Mono
# COPY ./VINS-Mono $CATKIN_WS/src/VINS-Mono/
# # use the following line if you only have this dockerfile
# # RUN git clone https://github.com/HKUST-Aerial-Robotics/VINS-Mono.git

# # Build VINS-Mono
# WORKDIR $CATKIN_WS
# ENV TERM xterm
# ENV PYTHONIOENCODING UTF-8
# RUN catkin config \
#       --extend /opt/ros/$ROS_DISTRO \
#       --cmake-args \
#         -DCMAKE_BUILD_TYPE=Release && \
#     catkin build && \
#     sed -i '/exec "$@"/i \
#             source "/root/catkin_ws/devel/setup.bash"' /ros_entrypoint.sh

#to do 
# git config --global user.name "wzYounger"
# git config --global user.email "2552846766@qq.com"
# git config --global --add safe.directory /home/cvrsg/gitee/plvins_catkinws/src/PL-VINS