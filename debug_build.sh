#!/bin/bash
#
# /usr/bin/cmake --no-warn-unused-cli -DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=TRUE -DCMAKE_BUILD_TYPE:STRING=Debug -DCMAKE_C_COMPILER:FILEPATH=/bin/x86_64-linux-gnu-gcc-8 -H./ -B./build -G "Unix Makefiles"
# /usr/bin/cmake --build build --config Debug --target mono_upper_slam -j 6 --
# /usr/bin/cmake --no-warn-unused-cli -DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=TRUE -DCMAKE_BUILD_TYPE:STRING=Debug -DCMAKE_C_COMPILER:FILEPATH=/usr/bin/gcc-7 -DCMAKE_CXX_COMPILER:FILEPATH=/usr/bin/g++-7 -H./ -B./build -G "Unix Makefiles"
# /usr/bin/cmake --build build --config Debug --target mono_upper_slam -- -j 16
# /usr/bin/cmake --build build --config Debug --target extrator_test -- -j 16

source /opt/ros/noetic/setup.bash
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Debug ..
make -j32
