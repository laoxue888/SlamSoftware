echo "Configuring and building Thirdparty/DBoW2 ..."

cd /root/workspace/slamsoftware/thirdparty/ORB_SLAM2/Thirdparty/DBoW2
if [ ! -d "build" ]; then
    mkdir build
else
    rm -rf build/*
fi
cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug
make -j

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."

if [ ! -d "build" ]; then
    mkdir build
else
    rm -rf build/*
fi
cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug
make -j

cd ../../../

echo "Uncompress vocabulary ..."

cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..

echo "Configuring and building ORB_SLAM2 ..."

if [ ! -d "build" ]; then
    mkdir build
else
    rm -rf build/*
fi
cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug
make -j
