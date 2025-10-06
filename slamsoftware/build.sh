#!/bin/bash
# rm -rf ./build
# mkdir build
# cd build
# cmake .. -DCMAKE_BUILD_TYPE=Debug
# make -j4

cd /root/workspace/slamsoftware

# if [ ! -d "build" ]; then
#     mkdir build
# else
#     rm -rf build/*
# fi
# cd build
# Local_Dir=$(cd "$(dirname "$0")"; pwd)
# echo "Now work at Dir:$Local_Dir"
# cmake .. -DCMAKE_BUILD_TYPE=Debug
# make

# 转换ui文件
cd ui
/opt/Qt/6.9.2/gcc_64/libexec/uic mainwindow.ui -o ../include/ui_mainwindow.h

# 编译代码
/opt/Qt/Tools/CMake/bin/cmake --build /root/workspace/slamsoftware/build/Desktop_Qt_6_9_2-Debug --target all