# FASTLIO2 ROS2

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
    >   ```shell
    >   git clone https://github.com/Livox-SDK/Livox-SDK2.git
    >   cd Livox-SDK2/
    >   mkdir -p build && cd build
    >   cmake .. && make -j4
    >   sudo make install
    >   ```
    >
    > - `Sophus`
    >
    >   ```shell
    >   git clone https://github.com/strasdat/Sophus.git
    >   cd Sophus
    >   git checkout 1.22.10
    >   mkdir -p build && cd build
    >   cmake .. -DSOPHUS_USE_BASIC_LOGGING=ON && make -j4
    >   sudo make install
    >   ```
    >
    > - `gtsam`
    >
    >   ```shell
    >   git clone https://github.com/borglab/gtsam.git
    >   cd gtsam
    >   mkdir -p build && cd build
    >   cmake .. && make -j4
    >   sudo make install
    >   ```
    >
    > - `livox_ros_driver2`
    >
    >   ```shell
    >   git clone https://github.com/Livox-SDK/livox_ros_driver2.git src/livox_ros_driver2
    >   cd src/livox_ros_driver2
    >   source /opt/ros/humble/setup.sh
    >   bash build.sh humble
    >   ```

## 实例数据集
```text
链接: https://pan.baidu.com/s/1rTTUlVwxi1ZNo7ZmcpEZ7A?pwd=t6yb 提取码: t6yb 
--来自百度网盘超级会员v7的分享
```

## 部分脚本

### 1.激光惯性里程计 
```shell
ros2 launch fastlio2 lio_launch.py
ros2 bag play your_bag_file
```

### 2.里程计加回环
#### 启动回环节点
```shell
ros2 launch pgo pgo_launch.py
ros2 bag play your_bag_file
```
#### 保存地图
```shell
ros2 service call /pgo/save_maps interface/srv/SaveMaps "{file_path: 'your_save_dir', save_patches: true}"
```

### 3.里程计加重定位
#### 启动重定位节点
```shell
ros2 launch localizer localizer_launch.py
ros2 bag play your_bag_file // 可选
```
#### 设置重定位初始值
```shell
ros2 service call /localizer/relocalize interface/srv/Relocalize "{"pcd_path": "your_map.pcd", "x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0, "pitch": 0.0, "roll": 0.0}"
```
#### 检查重定位结果
```shell
ros2 service call /localizer/relocalize_check interface/srv/IsValid "{"code": 0}"
```

### 4.一致性地图优化
#### 启动一致性地图优化节点
```shell
ros2 launch hba hba_launch.py
```
#### 调用优化服务
```shell
ros2 service call /hba/refine_map interface/srv/RefineMap "{"maps_path": "your maps directory"}"
```
**如果需要调用优化服务，保存地图时需要设置save_patches为true**

## Acknowledge

1. https://github.com/hku-mars/FAST_LIO
2. https://github.com/Livox-SDK/Livox-SDK2
3. https://github.com/Livox-SDK/livox_ros_driver2
4. https://github.com/strasdat/Sophus
