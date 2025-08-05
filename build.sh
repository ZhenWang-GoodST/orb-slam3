echo "Configuring and building Thirdparty/DBoW2 ..."

cd Thirdparty/DBoW2
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j16

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."
echo "-----------------------------------------"
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j16

# cd ../../utils

# echo "Configuring and building Thirdparty/g2o ..."
# echo "-----------------------------------------"
# mkdir build
# cd build
# cmake .. -DCMAKE_BUILD_TYPE=Release
# make -j4

cd ../../../

echo "Uncompress vocabulary ..."
echo "-----------------------------------------"
cd Vocabulary
#tar -xf ORBvoc.txt.tar.gz
cd ..

echo "Configuring and building ORB_SLAM3 ..."
echo "-----------------------------------------"
mkdir buildrelease
cd buildrelease
source /opt/ros/noetic/setup.bash
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j16
