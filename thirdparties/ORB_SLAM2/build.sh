#!/bin/bash
# 注意，编译的时候，把所有的conda环境都取消激活

echo "Configuring and building DBoW2 ..."

cd /root/workspace/thirdparties/DBoW2
if [ ! -d "build" ]; then
    mkdir build
else
    rm -rf build/*
fi
cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug
make -j

cd ../../g2o

echo "Configuring and building g2o ..."

if [ ! -d "build" ]; then
    mkdir build
else
    rm -rf build/*
fi
cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug
make -j

cd ../../Vocabulary

echo "Uncompress vocabulary ..."

tar -xf ORBvoc.txt.tar.gz


cd ../ORB_SLAM2

# export OPENGL_glu_LIBRARY=/usr/lib/x86_64-linux-gnu/libGL.so
# export OPENGL_gl_LIBRARY=/usr/lib/x86_64-linux-gnu/libGL.so
# export OPENGL_INCLUDE_DIR=/usr/include/GL

# cd /root/workspace/slamsoftware/thirdparty/ORB_SLAM2

echo "Configuring and building ORB_SLAM2 ..."

if [ ! -d "build" ]; then
    mkdir build
else
    rm -rf build/*
fi
cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug
make -j
