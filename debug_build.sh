#!/bin/bash
# /usr/bin/cmake --no-warn-unused-cli -DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=TRUE -DCMAKE_BUILD_TYPE:STRING=Debug -DCMAKE_C_COMPILER:FILEPATH=/bin/x86_64-linux-gnu-gcc-8 -H./ -B./build -G "Unix Makefiles"
#/usr/bin/cmake --build build --config Debug --target mono_upper_slam -j 6 --
# /usr/bin/cmake --no-warn-unused-cli -DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=TRUE -DCMAKE_BUILD_TYPE:STRING=Debug -DCMAKE_C_COMPILER:FILEPATH=/usr/bin/x86_64-linux-gnu-gcc-7 -H./ -B./build -G "Unix Makefiles"
/usr/bin/cmake --build build --config Debug --target mono_upper_slam -- -j 16
