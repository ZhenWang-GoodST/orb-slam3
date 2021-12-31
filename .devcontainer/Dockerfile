FROM ros:noetic-ros-base

# =========================================================
# =========================================================

# Are you are looking for how to use this docker file?
#   - https://docs.openvins.com/dev-docker.html
#   - https://docs.docker.com/get-started/
#   - http://wiki.ros.org/docker/Tutorials/Docker

# =========================================================
# =========================================================

# Dependencies we use, catkin tools is very good build system
# Also some helper utitiles for fast in terminal edits (nano etc)
# WORKDIR /home/wz/docker/files-in-image
# COPY ./necessary/ /usr/local/ 错误，代表通配符，只复制了文件
COPY ./new/ /usr/local/
RUN sudo chmod 777 /tmp && \
    adduser wz && \
    usermod -aG sudo wz && \
    chown root:root /usr/bin/sudo && \
    chmod 4755 /usr/bin/sudo &&\
    apt-get update && apt-get install -y libeigen3-dev libceres-dev libflann-dev \
    libglew-dev libegl1-mesa-dev\ 
    libarpack* \
    libvtk7-dev \
    libgtk2.0-dev pkg-config \
    ros-noetic-cv-bridge libgflags-dev libunwind-dev gdb git cmake libgsl-dev \
    && rm -rf /var/lib/apt/lists/*
# USER wz 
    # ros-noetic-pcl-ros
    # echo root:123456 | chapasswd &&
# RUN sudo chmod 777 /tmp && apt-get update && apt-get install -y libeigen3-dev libceres-dev libflann-dev \
    # libglew-dev libegl1-mesa-dev\ 
    # libarpack* \
    # libvtk7-dev \
    # libgtk2.0-dev pkg-config \ opencv
    # ros-noetic-cv-bridge libgflags-dev libunwind-dev gdb  git cmake \
    # && rm -rf /var/lib/apt/lists/* 
# RUN sudo apt-get install -y python3-catkin-tools python3-osrf-pycommon
# #json pangonlin glog opencv pcl

# # Install deps needed for clion remote debugging
# # https://blog.jetbrains.com/clion/2020/01/using-docker-with-clion/
# RUN apt-get update && apt-get install -y ssh build-essential gcc g++ \
#     gdb clang cmake rsync tar python && apt-get clean
# RUN ( \
#     echo 'LogLevel DEBUG2'; \
#     echo 'PermitRootLogin yes'; \
#     echo 'PasswordAuthentication yes'; \
#     echo 'Subsystem sftp /usr/lib/openssh/sftp-server'; \
#   ) > /etc/ssh/sshd_config_test_clion \
#   && mkdir /run/sshd
# RUN useradd -m user && yes password | passwd user
# RUN usermod -s /bin/bash user
# CMD ["/usr/sbin/sshd", "-D", "-e", "-f", "/etc/ssh/sshd_config_test_clion"]

# 1  cd home/tonglu/github/script/
#     2  vim build_opencv.sh 
#     3  vi build_opencv.sh 
#     4  ./build_opencv.sh 
#     5  cd ../opencv/orb_build/
#     6  rm -rf ./*
#     7  cd -
#     8  ./build_opencv.sh 
#     9  cd ../pcl
#    10  cd orbbuild/
#    11  make -j32
#    12  cmake ..
#    13  apt-get install libvtk7.1
#    14  apt-get install libvtk6-dev
#    15  apt-get install libvtk7.1
#    16  cmake ..
#    17  apt-get install libvtk7-dev
#    18  cmake .
#    19  make -j32
#    20  cmake ..
#    21  make -j32
#    22  apt-get install libflann-dev
#    23  make -j32
#    24  cmake ..
#    25  make -j32
#    26  rm -rf ./*
#    27  cmake ..
#    28  make -j16
#    29  make install
#    30  cd ../../
#    31  cd ../VO-LOAM/test/
#    32  cd orb-slam3/
#    33  ./clean.sh 
#    34  ./debug_build.sh 
#    35  ./build
#    36  ./build.sh 
#    37  apt-get install libceres-dev
#    38  ./build.sh 
#    39  source /opt/ros/noetic/setup.bash 
#    40  ./build.sh 
#    41  apt-get install ros-noetic-roscpp
#    42  ./build.sh 
#    43  ./clean.sh 
#    44  ./build.sh 
#    45  apt-get install ros-noetic-roscpp
#    46  apt-cache search ros-noetic-roscpp
#    47  ./build.sh 
#    48  roscd roscpp
#    49  ls
#    50  cd cmake/
#    51  ls
#    52  ./build.sh 
#    53  cd -
#    54  ./build.sh 
#    55  cd -
#    56  cd /home/tonglu/VO-LOAM/test/orb-slam3/
#    57  ./build.sh 
#    58  cd -
#    59  ./build.sh 
#    60  ./clean.sh 
#    61  ./build.sh 
#    62  cd -
#    63  ls
#    64  cd -
#    65  ./build.sh 
#    66  source /opt/ros/noetic/setup.bash 
#    67  ./build.sh 
#    68  ./clean.sh `
#    69  ./clean.sh 
#    70  ./build.sh 
#    71  cat build.sh 
#    72  cd buildrelease/
#    73  make -j16
#    74  cmake ..
#    75  rm -rf ./*
#    76  cmake ..
#    77  rm -rf ./*
#    78  cmake ..
#    79  rm -rf ./*
#    80  cmake ..
#    81  rm -rf ./*
#    82  cmake ..
#    83  rm -rf ./*
#    84  cmake ..
#    85  apt-get remove ros-noetic-roscpp*
#    86  apt-get install ros-noetic-roscpp*
#    87  source /opt/ros/noetic/setup.bash 
#    88  cmake ..
#    89  rm -rf ./*
#    90  cmake ..
#    91  rm -rf ./*
#    92  cmake ..
#    93  cd /home/tonglu/github/catkindir/catkin
#    94  ls
#    95  catkin_make
#    96  cd -
#    97  rm -rf ./*
#    98  cmake ..
#    99  make 
#   100  cmake ..
#   101  cat /opt/ros/noetic/share/catkin/cmake/catkinConfig.cmake
#   102  cmake ..
#   103  cd -
#   104  catkin_make
#   105  rm -rf build* devel*
#   106  catkin_make
#   107  rm -rf build* devel*
#   108  catkin_make
#   109  source /opt/ros/noetic/setup.bash 
#   110  catkin_make
#   111  exit
#   112  cd home/tonglu/github/catkindir/catkin/
#   113  ls
#   114  rm -rf build* devel*
#   115  ls
#   116  catkin_m
#   117  source /opt/ros/noetic/setup.bash 
#   118  catkin_make
#   119  cd /home/tonglu/VO-LOAM/test/orb-slam3/buildrelease/
#   120  cmake ..
#   121  rm -rf ./*I
#   122  rm -rf ./*
#   123  ls
#   124  cmake ..
#   125  rm -rf ./*
#   126  ls
#   127  cmake ..
#   128  apt-get install ros-noetic-pcl-ros
#   129  cmake ..
#   130  make -j16
#   131  cd /home/tonglu/github/json/
#   132  mkdir orbbuild
#   133  cd orbbuild/
#   134  cmake ..
#   135  make -j
#   136  make install
#   137  cd /home/tonglu/VO-LOAM/test/orb-slam3/buildrelease
#   138  cmake ..
#   139  make -j16
#   140  sudo apt install libarpack*
#   141  apt-get install ros-noetic-cv-bridge
#   142  make -j16
#   143  cmake ..
#   144  make -j16
#   145  cmake ..
#   146  make -j16
#   147  rm -rf ./*
#   148  cmake ..
#   149  make -j16
#   150  cmake ..
#   151  make -j16
#   152  history 

