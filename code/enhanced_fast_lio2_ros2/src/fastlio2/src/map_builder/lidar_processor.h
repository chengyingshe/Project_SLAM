#pragma once
#include "commons.h"
#include "ieskf.h"
#include "ikd_Tree.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

struct LocalMap
{
    bool initialed = false;
    BoxPointType local_map_corner;
    Vec<BoxPointType> cub_to_rm;
};

// 添加退化检测相关参数设置
class DegeneracyOptions {
public:
    bool check_rotation = true;  // 是否检测旋转退化
    bool check_translation = true;  // 是否检测平移退化
    double rotation_threshold = 0.1;  // 旋转退化阈值
    double translation_threshold = 0.1;  // 平移退化阈值
};

class LidarProcessor
{
public:
    LidarProcessor(Config &config, std::shared_ptr<IESKF> kf);

    void trimCloudMap();
    void incrCloudMap();
    void initCloudMap(PointVec &point_vec);
    void process(SyncPackage &package);
    void updateLossFunc(State &state, SharedState &share_data);
    static CloudType::Ptr transformCloud(CloudType::Ptr inp, const M3D &r, const V3D &t);
    M3D r_wl() { return m_kf->x().r_wi * m_kf->x().r_il; }
    V3D t_wl() { return m_kf->x().t_wi + m_kf->x().r_wi * m_kf->x().t_il; }

private:
    // 添加退化检测相关函数
    bool checkDegeneracy(const Eigen::MatrixXd& J, const Eigen::VectorXd& residuals);
    void analyzeDegeneracy(const Eigen::MatrixXd& H, DegeneracyOptions& options);
    void adjustWeightsByDegeneracy(Eigen::MatrixXd& H, Eigen::VectorXd& b);

    Config m_config;
    LocalMap m_local_map;
    std::shared_ptr<IESKF> m_kf;
    std::shared_ptr<KD_TREE<PointType>> m_ikdtree;
    CloudType::Ptr m_cloud_lidar;
    CloudType::Ptr m_cloud_down_lidar;
    CloudType::Ptr m_cloud_down_world;
    std::vector<bool> m_point_selected_flag;
    CloudType::Ptr m_norm_vec;
    CloudType::Ptr m_effect_cloud_lidar;
    CloudType::Ptr m_effect_norm_vec;
    std::vector<PointVec> m_nearest_points;
    pcl::VoxelGrid<PointType> m_scan_filter;

    // 添加退化检测相关成员变量
    DegeneracyOptions m_degeneracy_options;
    bool m_is_degenerate;
    Eigen::Matrix<double, 6, 6> m_degeneracy_matrix;
};