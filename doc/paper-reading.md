# Paper Reading

## VPE-SLAM

来源: 2024 ICRA

论文链接: https://ieeexplore.ieee.org/document/10610865/

Github: [https://github.com/NeuCV-IRMI/VPE-SLAM](https://github.com/NeuCV-IRMI/VPE-SLAM) (未开源)

Reference:

```bib
@INPROCEEDINGS{10610865,
  author={Zhang, Zhiyao and Zhang, Yunzhou and Shen, You and Rong, Lei and Wang, Sizhan and Ouyang, Xin and Li, Yulong},
  booktitle={2024 IEEE International Conference on Robotics and Automation (ICRA)}, 
  title={VPE-SLAM: Neural Implicit Voxel-permutohedral Encoding for SLAM}, 
  year={2024},
  volume={},
  number={},
  pages={5104-5110},
  keywords={Geometry;Bundle adjustment;Simultaneous localization and mapping;Accuracy;Source coding;Robot vision systems;Distortion},
  doi={10.1109/ICRA57147.2024.10610865}}
```

优点：

1. 高精度。NeRF在室内场景重建时存在场景几何畸变的问题，该论文提出了一种新的方法能够对于室内场景产生高质量重建结果

参考：应用当前论文中的方法来改进LONER中NeRF点云重建方法，提高室外场景重建的精度；应用Voxelization体素化的方法来降低计算复杂度

## Degenerate Detection

来源: 2024 ICRA

论文链接: https://ieeexplore.ieee.org/document/10610340/

Github: https://github.com/jisehua/Degenerate-Detection

Reference:

```bib
@article{Ji2024APD,
  title={A Point-to-distribution Degeneracy Detection Factor for LiDAR SLAM using Local Geometric Models},
  author={Sehua Ji, Weinan Chen, Zerong Su, Yisheng Guan, Jiehao Li, Hong Zhang, Haifei Zhu},
  journal={2024 IEEE International Conference on Robotics and Automation (ICRA)},
  year={2024},
  pages={12283-12289},
}
```

优点：

1. 精度高、鲁棒性好。在走廊、隧道等特殊场景下能提供更好的重建结果

参考：应用论文中提出的局部几何模型到LONER中，测试实际精度提升

## FAST-LIO2

来源: IEEE Transactions on Robotics

论文链接: https://ieeexplore.ieee.org/document/9697912

Github: https://github.com/hku-mars/FAST_LIO

Reference:

```bib
@ARTICLE{9697912,
  author={Xu, Wei and Cai, Yixi and He, Dongjiao and Lin, Jiarong and Zhang, Fu},
  journal={IEEE Transactions on Robotics}, 
  title={FAST-LIO2: Fast Direct LiDAR-Inertial Odometry}, 
  year={2022},
  volume={38},
  number={4},
  pages={2053-2073},
  keywords={Laser radar;Robots;Real-time systems;Feature extraction;Data structures;Point cloud compression;Kalman filters;Aerial systems;sensor fusion;simultaneous localization and mapping (SLAM)},
  doi={10.1109/TRO.2022.3141876}}
```

优点：

1. 速度快。在LiDAR数据量较大的情况下，FAST-LIO2可以快速地完成SLAM，无需等待LiDAR数据全部接收完毕，从而节省了时间。

参考：应用论文中提出的ikd-Tree来优化LONER中的SLAM前端算法，加快处理速度