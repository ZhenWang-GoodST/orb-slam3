path=/home/cvrsg/VO-LOAM/github/orb-slam3
echo "Configuring and building Thirdparty/DBoW2 ..."
rm -rf ${path}/Thirdparty/DBoW2/build/*
rm -rf ${path}/Thirdparty/DBoW2/lib*

echo "Configuring and building Thirdparty/g2o ..."

rm -rf ${path}/Thirdparty/g2o/build/*
rm -rf ${path}/Thirdparty/g2o/lib*

echo "Configuring and building ORB_SLAM3 ..."

cd ${path}/buildrelease
rm -rf ${path}/buildrelease/*
