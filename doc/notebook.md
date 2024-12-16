## LONER论文详解

### 原因

1. 这篇论文仅使用了LiDAR数据实现实时定位和建图，与我们本次实验的目的相同
2. 论文中首次使用神经网络，利用有监督的方式训练模型来代替传统的SLAM算法，并在NICE-SLAM数据集上达到了SOTA

### 目标

1. 尝试替代论文中提出的神经网络模型中的模块，并进行训练和测试查看实际效果
2. 在上一步的基础上尝试修改或添加新的Loss Function，让模型达到更好的收敛效果
3. 尝试添加多模态，扩展原本的神经网络，加入IMU、RGB等数据，提升模型鲁棒性

## 学习笔记

> 参考论文中的 `Related Work`学习SLAM中的相关知识，并了解论文中提出的对比模型，总结论文中提出的创新点，为后续改进做准备

### LiDAR SLAM

LOAM（Lidar Odometry and Mapping）算法：该算法通过将SLAM问题分为两个部分来处理，高频率的里程计和低频率的建图。LOAM 是许多流行基于LiDAR的SLAM系统的基础

算法步骤：

1. **传感器数据采集** ：首先，通过激光雷达和IMU采集传感器数据。激光雷达负责获取周围环境的点云数据，IMU负责获取机器人的姿态和速度信息。
2. **点云预处理** ：对激光雷达采集到的点云数据进行预处理，包括去除噪声、填充空洞等。
3. **特征点提取** ：从预处理后的点云数据中提取特征点，如边缘、角点等。
4. **帧间匹配** ：根据特征点进行帧间匹配，计算激光雷达扫描之间的相对位姿。
5. **里程计计算** ：利用匹配结果和IMU数据，计算激光雷达扫描之间的相对位姿，从而实现高频率的里程计计算。
6. **建图** ：根据匹配结果和里程计计算得到的位姿，将激光雷达扫描数据投影到地图上，实现低频率的建图。
7. **回环检测** ：通过检测地图上的回环，校正定位和建图过程中的误差。
8. **实时定位** ：根据地图和激光雷达数据，实时计算机器人在地图上的位置。

---

Zhang, J., & Singh, S. (2014). LOAM: Lidar Odometry and Mapping in Real-time.  *Robotics: Science and Systems* .

#### ICP Alignment Algorithm

### NeRF-based SLAM

iMAP (Implicit Mapping and Positioning in Real-Time) ：

* **原理** ：iMAP使用一个多层感知器（MLP）网络来存储地图，并实时解码空间点的颜色和体积密度。这种方法依赖于神经隐式环境表示和体积渲染函数。
* **步骤** ：
  1. 使用MLP网络存储环境的隐式表示。
  2. 实时解码空间点的颜色和体积密度。
  3. 通过逆渲染过程优化相机姿态和MLP网络参数，以最小化光度误差。

NICE-SLAM (Neural Implicit Scalable Encoding for SLAM) ：

* **原理** ：NICE-SLAM通过粗到细的方法表示更大的场景。它结合了显式和隐式表达方法，使用体素网格显式存储特征向量。
* **步骤** ：
  1. 使用体素网格存储特征向量。
  2. 从体素或点云中提取特征向量，并通过MLP解码渲染所需的参数。
  3. 结合NeRF映射与视觉里程计（VO）进行逆渲染，优化相机姿态和MLP网络参数。

---

Mildenhall, Ben, Pratul P. Srinivasan, Matthew Tancik, Jonathan T. Barron, Ravi Ramamoorthi, and Ren Ng. "NeRF: Representing Scenes as Neural Radiance Fields for View Synthesis." ECCV, 2020.

Sucar, Edgar, Shikun Liu, Joseph Ortiz, and Andrew J. Davison. "iMAP: Implicit Mapping and Positioning in Real-Time." Proceedings of the IEEE/CVF International Conference on Computer Vision (ICCV), 2022.

Zhu, Zihan, Songyou Peng, Viktor Larsson, Weiwei Xu, Hujun Bao, Zhaopeng Cui, Martin R. Oswald, and Marc Pollefeys. "NICE-SLAM: Neural Implicit Scalable Encoding for SLAM." Proceedings of the IEEE/CVF Conference on Computer Vision and Pattern Recognition (CVPR), 2022.

### Methodology

> 讲述论文中提出的算法细节，包括模型架构、Loss Function、训练和测试流程等，详细说明论文中提出的改进

### Development

> 基于上面的总结，提出可能的改进方案
