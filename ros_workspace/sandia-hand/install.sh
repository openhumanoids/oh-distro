rm -r build
mkdir build
cd build
#echo $PWD/sandia-hand/build
cmake -DCMAKE_INSTALL_PREFIX=$PWD/build  ../
make -j8 install
