#!/bin/bash

echo ""
echo "****************** Clone repository Livox-SDK2 and build it ******************"
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2/
mkdir -p build && cd build
cmake .. && make -j4
sudo make install

cd ../../

echo ""
echo "****************** Clone repository Sophus and build it ******************"
git clone https://github.com/strasdat/Sophus.git
cd Sophus
git checkout 1.22.10
mkdir -p build && cd build
cmake .. -DSOPHUS_USE_BASIC_LOGGING=ON && make -j4
sudo make install

cd ../../

echo ""
echo "****************** Clone repository gtsam and build it ******************"
git clone https://github.com/borglab/gtsam.git
cd gtsam
mkdir -p build && cd build
cmake .. && make -j4
sudo make install

cd ../../

echo ""
echo "****************** Start to build fast_lio2_ros2 ******************"
cd fast_lio2_ros2/src/livox_ros_driver2
source /opt/ros/humble/setup.sh
bash build.sh humble

cd ../../../

echo ""
echo "****************** Start to build enhanced_fast_lio2_ros2 ******************"
cd enhanced_fast_lio2_ros2/src/livox_ros_driver2
source /opt/ros/humble/setup.sh
bash build.sh humble
