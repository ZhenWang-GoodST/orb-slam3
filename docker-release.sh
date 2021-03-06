echo "Configuring and building Thirdparty/DBoW2 ..."

cd Thirdparty/DBoW2
mkdir dockerrelease
cd dockerrelease
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."
echo "-----------------------------------------"
mkdir dockerrelease
cd dockerrelease
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4

# cd ../../..

# echo "Configuring and building Thirdparty/g2o ..."
# echo "-----------------------------------------"
# mkdir dockerrelease
# cd dockerrelease
# cmake .. -DCMAKE_BUILD_TYPE=Release
# make -j4
