echo "Configuring and building Thirdparty/DBoW2 ..."

cd /root/workspace/slamsoftware/thirdparty/DBoW2
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

cd ../../

echo "Uncompress vocabulary ..."

cd Vocabulary
tar -xf ORBvoc.txt.tar.gz


echo "Configuring and building ORB_SLAM2 ..."

cd ../ORB_SLAM2

if [ ! -d "build" ]; then
    mkdir build
else
    rm -rf build/*
fi
cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug
make -j

# echo "Configuring and building ORB_SLAM3 ..."

# cd ../../ORB_SLAM3

# if [ ! -d "build" ]; then
#     mkdir build
# else
#     rm -rf build/*
# fi
# cd build
# cmake .. -DCMAKE_BUILD_TYPE=Debug
# make -j