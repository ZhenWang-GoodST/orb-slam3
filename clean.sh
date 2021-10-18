echo "Configuring and building Thirdparty/DBoW2 ..."

cd Thirdparty/DBoW2
cd build
rm -rf ./*
rm -rf ../lib*
cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."

cd build
rm -rf ./*
rm -rf ../lib*

cd ../../utils

echo "Configuring and building Thirdparty/g2o ..."

cd build
rm -rf ./*

cd ../../../


echo "Configuring and building ORB_SLAM3 ..."

cd buildrelease
rm -rf ./*
