# Project_SLAM

## Environments

1. Ubuntu 22.04
2. ROS2 Humble

## Quick Start

1. Execute the script `install_env.sh` to prepare the environment.

    ```shell
    bash install_env.sh
    ```

    > You could also download each repositories and build them manually:
    >
    > - `Livox-SDK2`
    >
    > ```shell
    > git clone https://github.com/Livox-SDK/Livox-SDK2.git
    > cd Livox-SDK2/
    > mkdir -p build && cd build
    > cmake .. && make -j4
    > sudo make install
    > ```
    >
    > - `Sophus`
    >
    > ```shell
    > git clone https://github.com/strasdat/Sophus.git
    > cd Sophus
    > git checkout 1.22.10
    > mkdir -p build && cd build
    > cmake .. -DSOPHUS_USE_BASIC_LOGGING=ON && make -j4
    > sudo make install
    > ```
    >
    > - `gtsam`
    >
    > ```shell
    > git clone https://github.com/borglab/gtsam.git
    > cd gtsam
    > mkdir -p build && cd build
    > cmake .. && make -j4
    > sudo make install
    > ```
    >
    > - `livox_ros_driver2`
    >
    > ```shell
    > cd src/livox_ros_driver2
    > source /opt/ros/humble/setup.sh
    > bash build.sh humble  # it'll build all packages in `src/`
    > ```

2. Download the `ros2bag` from [Baidu Drive](https://pan.baidu.com/s/1YKzqZwJ028OCWlzXE_jpKg?pwd=1234) or [Google Drive](https://drive.google.com/file/d/1n9w-Rf8kakgsKgI345gRvSOBWSkfqdtr/view?usp=sharing), and then move it to `code/ros2bag/` and extract it:

   ```shell
   tar -zxvf ros2bag_with_gt.tar.gz
   ```

3. Run `fast_lio2_ros2_v1`

   ```shell
   cd fast_lio2_ros2

   # terminal 1
   source install/setup.bash
   ln -s ../ros2bag/ros2bag_with_gt
   ros2 bag play <ros2-bag>
   
   # terminal 2
   source install/setup.bash
   ros2 launch fastlio2 lio_launch.py
   ```

4. Run `fast_lio2_ros2_v2` (with [Degenerate-Detection](https://github.com/jisehua/Degenerate-Detection))

   ```shell
   cd fast_lio2_ros2

   # terminal 1
   source install/setup.bash
   ros2 bag play <ros2-bag>
   
   # terminal 2
   source install/setup.bash
   ros2 launch fastlio2 fastlio2.launch.py
   ```

5. Run `fast_lio2_ros2_v3` (with `SC-PGO`)

   ```shell
   cd enhanced_fast_lio2_ros2

   # terminal 1
   source install/setup.bash
   ros2 bag play <ros2-bag>
   
   # terminal 2
   source install/setup.bash
   ros2 launch fastlio2 fastlio2_pgo.launch.py
   ```

## Testing Data

- You can download the **ros1bag** provided by [FAST_LIO](https://github.com/hku-mars/FAST_LIO?tab=readme-ov-file#4-rosbag-example) from [Google Drive](https://drive.google.com/drive/folders/1CGYEJ9-wWjr8INyan6q1BZz_5VtGB-fP?usp=sharing), and then convert them to `ros2bag` format using the scripts below:

  ```shell
  pip install rosbags
  rosbags-convert --src <ros1-bag> --dst <ros2-bag>
  ```

## Evaluation

1. Install `evo`:

   ```shell
   pip install evo
   ```

2. `evo_ape`: Absolute Pose Error

   ```shell
   evo_ape kitti gt.kitti target.kitti -p --save_results results/ape.zip
   ```

3. `evo_rpe`: Relative Pose Error

   ```shell
   evo_rpe kitti gt.kitti target.kitti -va -p --save_results results/rpe.zip
   ```

4. Process multiple results from a metric:

   ```shell
   evo_res results/*.zip -p --save_table results/table.csv
   ```

## Acknowledge

- https://github.com/hku-mars/FAST_LIO
- https://github.com/Livox-SDK/Livox-SDK2
- https://github.com/Livox-SDK/livox_ros_driver2
- https://github.com/strasdat/Sophus
- https://github.com/liangheming/FASTLIO2_ROS2
- https://github.com/jisehua/Degenerate-Detection
- https://github.com/MichaelGrupp/evo
