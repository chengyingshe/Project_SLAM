#include "lidar_processor.h"

void LidarProcessor::analyzeDegeneracy(const Eigen::MatrixXd& H, DegeneracyOptions& options) {
    // 对信息矩阵进行特征值分解
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eigensolver(H.block<6,6>(0,0));
    Eigen::Matrix<double, 6, 1> eigenvalues = eigensolver.eigenvalues();
    Eigen::Matrix<double, 6, 6> eigenvectors = eigensolver.eigenvectors();

    // 分析旋转和平移部分的特征值
    double min_rotation_eigenvalue = eigenvalues.head(3).minCoeff();
    double min_translation_eigenvalue = eigenvalues.tail(3).minCoeff();

    // 更新退化状态
    if(options.check_rotation && min_rotation_eigenvalue < options.rotation_threshold) {
        m_is_degenerate = true;
        // 存储退化方向信息
        m_degeneracy_matrix.block<3,3>(0,0) = eigenvectors.block<3,3>(0,0);
    }

    if(options.check_translation && min_translation_eigenvalue < options.translation_threshold) {
        m_is_degenerate = true;
        // 存储退化方向信息
        m_degeneracy_matrix.block<3,3>(3,3) = eigenvectors.block<3,3>(3,3);
    }
}

void LidarProcessor::adjustWeightsByDegeneracy(Eigen::MatrixXd& H, Eigen::VectorXd& b) {
    if(!m_is_degenerate) return;

    // 计算权重矩阵
    Eigen::Matrix<double, 6, 6> weight_matrix = Eigen::Matrix<double, 6, 6>::Identity();
    
    // 根据退化方向调整权重
    for(int i = 0; i < 6; i++) {
        if(m_degeneracy_matrix.col(i).norm() > 0.1) {
            weight_matrix(i,i) = 0.01;  // 对退化方向使用较小的权重
        }
    }

    // 应用权重
    H = weight_matrix * H;
    b = weight_matrix * b;
}

LidarProcessor::LidarProcessor(Config &config, std::shared_ptr<IESKF> kf) : m_config(config), m_kf(kf)
{
    m_ikdtree = std::make_shared<KD_TREE<PointType>>();
    m_ikdtree->set_downsample_param(m_config.map_resolution);
    m_cloud_down_lidar.reset(new CloudType);
    m_cloud_down_world.reset(new CloudType(10000, 1));
    m_norm_vec.reset(new CloudType(10000, 1));
    m_effect_cloud_lidar.reset(new CloudType(10000, 1));
    m_effect_norm_vec.reset(new CloudType(10000, 1));
    m_nearest_points.resize(10000);
    m_point_selected_flag.resize(10000, false);

    if (m_config.scan_resolution > 0.0)
    {
        m_scan_filter.setLeafSize(m_config.scan_resolution, m_config.scan_resolution, m_config.scan_resolution);
    }

    m_kf->setLossFunction([&](State &s, SharedState &d)
                          { updateLossFunc(s, d); });
    m_kf->setStopFunction([&](const V21D &delta) -> bool
                          { V3D rot_delta = delta.block<3, 1>(0, 0);
                            V3D t_delta = delta.block<3, 1>(3, 0);
                            return (rot_delta.norm() * 57.3 < 0.01) && (t_delta.norm() * 100 < 0.015); });

    // 初始化退化检测相关参数
    m_is_degenerate = false;
    m_degeneracy_matrix.setIdentity();
    m_degeneracy_options.check_rotation = true;
    m_degeneracy_options.check_translation = true;
    m_degeneracy_options.rotation_threshold = 0.1;
    m_degeneracy_options.translation_threshold = 0.1;
}

void LidarProcessor::trimCloudMap()
{
    m_local_map.cub_to_rm.clear();
    const State &state = m_kf->x();
    Eigen::Vector3d pos_lidar = state.t_wi + state.r_wi * state.t_il;

    if (!m_local_map.initialed)
    {
        for (int i = 0; i < 3; i++)
        {
            m_local_map.local_map_corner.vertex_min[i] = pos_lidar[i] - m_config.cube_len / 2.0;
            m_local_map.local_map_corner.vertex_max[i] = pos_lidar[i] + m_config.cube_len / 2.0;
        }
        m_local_map.initialed = true;
        return;
    }
    float dist_to_map_edge[3][2];
    bool need_move = false;
    double det_thresh = m_config.move_thresh * m_config.det_range;
    for (int i = 0; i < 3; i++)
    {
        dist_to_map_edge[i][0] = fabs(pos_lidar(i) - m_local_map.local_map_corner.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_lidar(i) - m_local_map.local_map_corner.vertex_max[i]);

        if (dist_to_map_edge[i][0] <= det_thresh || dist_to_map_edge[i][1] <= det_thresh)
            need_move = true;
    }
    if (!need_move)
        return;
    BoxPointType new_corner, temp_corner;
    new_corner = m_local_map.local_map_corner;
    float mov_dist = std::max((m_config.cube_len - 2.0 * m_config.move_thresh * m_config.det_range) * 0.5 * 0.9, double(m_config.det_range * (m_config.move_thresh - 1)));

    for (int i = 0; i < 3; i++)
    {
        temp_corner = m_local_map.local_map_corner;
        if (dist_to_map_edge[i][0] <= det_thresh)
        {
            new_corner.vertex_max[i] -= mov_dist;
            new_corner.vertex_min[i] -= mov_dist;
            temp_corner.vertex_min[i] = m_local_map.local_map_corner.vertex_max[i] - mov_dist;
            m_local_map.cub_to_rm.push_back(temp_corner);
        }
        else if (dist_to_map_edge[i][1] <= det_thresh)
        {
            new_corner.vertex_max[i] += mov_dist;
            new_corner.vertex_min[i] += mov_dist;
            temp_corner.vertex_max[i] = m_local_map.local_map_corner.vertex_min[i] + mov_dist;
            m_local_map.cub_to_rm.push_back(temp_corner);
        }
    }
    m_local_map.local_map_corner = new_corner;

    PointVec points_history;
    m_ikdtree->acquire_removed_points(points_history);

    // 删除局部地图之外的点云
    if (m_local_map.cub_to_rm.size() > 0)
        m_ikdtree->Delete_Point_Boxes(m_local_map.cub_to_rm);
    return;
}

void LidarProcessor::incrCloudMap()
{
    if (m_cloud_down_lidar->empty())
        return;
    const State &state = m_kf->x();
    int size = m_cloud_down_lidar->size();
    PointVec point_to_add;
    PointVec point_no_need_downsample;
    for (int i = 0; i < size; i++)
    {
        const PointType &p = m_cloud_down_lidar->points[i];
        Eigen::Vector3d point(p.x, p.y, p.z);
        point = state.r_wi * (state.r_il * point + state.t_il) + state.t_wi;
        m_cloud_down_world->points[i].x = point(0);
        m_cloud_down_world->points[i].y = point(1);
        m_cloud_down_world->points[i].z = point(2);
        m_cloud_down_world->points[i].intensity = m_cloud_down_lidar->points[i].intensity;
        // 如果该点附近没有近邻点则需要添加到地图中
        if (m_nearest_points[i].empty())
        {
            point_to_add.push_back(m_cloud_down_world->points[i]);
            continue;
        }

        const PointVec &points_near = m_nearest_points[i];
        bool need_add = true;
        PointType downsample_result, mid_point;
        mid_point.x = std::floor(m_cloud_down_world->points[i].x / m_config.map_resolution) * m_config.map_resolution + 0.5 * m_config.map_resolution;
        mid_point.y = std::floor(m_cloud_down_world->points[i].y / m_config.map_resolution) * m_config.map_resolution + 0.5 * m_config.map_resolution;
        mid_point.z = std::floor(m_cloud_down_world->points[i].z / m_config.map_resolution) * m_config.map_resolution + 0.5 * m_config.map_resolution;

        // 如果该点所在的voxel没有点，则直接加入地图，且不需要降采样
        if (fabs(points_near[0].x - mid_point.x) > 0.5 * m_config.map_resolution && fabs(points_near[0].y - mid_point.y) > 0.5 * m_config.map_resolution && fabs(points_near[0].z - mid_point.z) > 0.5 * m_config.map_resolution)
        {
            point_no_need_downsample.push_back(m_cloud_down_world->points[i]);
            continue;
        }
        float dist = sq_dist(m_cloud_down_world->points[i], mid_point);

        for (int readd_i = 0; readd_i < m_config.near_search_num; readd_i++)
        {
            // 如果该点的近邻点较少，则需要加入到地图中
            if (points_near.size() < static_cast<size_t>(m_config.near_search_num))
                break;
            // 如果该点的近邻点距离voxel中心点的距离比该点距离voxel中心点更近，则不需要加入该点
            if (sq_dist(points_near[readd_i], mid_point) < dist)
            {
                need_add = false;
                break;
            }
        }
        if (need_add)
            point_to_add.push_back(m_cloud_down_world->points[i]);
    }
    m_ikdtree->Add_Points(point_to_add, true);
    m_ikdtree->Add_Points(point_no_need_downsample, false);
}

void LidarProcessor::initCloudMap(PointVec &point_vec)
{
    m_ikdtree->Build(point_vec);
}

void LidarProcessor::process(SyncPackage &package)
{
    // m_kf->setLossFunction([&](State &s, SharedState &d)
    //                       { updateLossFunc(s, d); });
    // m_kf->setStopFunction([&](const V21D &delta) -> bool
    //                       { V3D rot_delta = delta.block<3, 1>(0, 0);
    //                         V3D t_delta = delta.block<3, 1>(3, 0);
    //                         return (rot_delta.norm() * 57.3 < 0.01) && (t_delta.norm() * 100 < 0.015); });
    if (m_config.scan_resolution > 0.0)
    {
        m_scan_filter.setInputCloud(package.cloud);
        m_scan_filter.filter(*m_cloud_down_lidar);
    }
    else
    {
        pcl::copyPointCloud(*package.cloud, *m_cloud_down_lidar);
    }
    trimCloudMap();
    m_kf->update();
    incrCloudMap();
}

void LidarProcessor::updateLossFunc(State &state, SharedState &share_data)
{
    int size = m_cloud_down_lidar->size();
#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
    for (int i = 0; i < size; i++) {
        //FAST-LIO2中的特征提取部分
        PointType &point_body = m_cloud_down_lidar->points[i];
        PointType &point_world = m_cloud_down_world->points[i];
        Eigen::Vector3d point_body_vec(point_body.x, point_body.y, point_body.z);
        Eigen::Vector3d point_world_vec = state.r_wi * (state.r_il * point_body_vec + state.t_il) + state.t_wi;
        point_world.x = point_world_vec(0);
        point_world.y = point_world_vec(1);
        point_world.z = point_world_vec(2);
        point_world.intensity = point_body.intensity;
        std::vector<float> point_sq_dist(m_config.near_search_num);
        auto &points_near = m_nearest_points[i];
        m_ikdtree->Nearest_Search(point_world, m_config.near_search_num, points_near, point_sq_dist);
        if (points_near.size() >= static_cast<size_t>(m_config.near_search_num) && point_sq_dist[m_config.near_search_num - 1] <= 5)
            m_point_selected_flag[i] = true;
        else
            m_point_selected_flag[i] = false;
        if (!m_point_selected_flag[i])
            continue;

        Eigen::Vector4d pabcd;
        m_point_selected_flag[i] = false;
        if (esti_plane(points_near, 0.1, pabcd))
        {
            double pd2 = pabcd(0) * point_world_vec(0) + pabcd(1) * point_world_vec(1) + pabcd(2) * point_world_vec(2) + pabcd(3);
            double s = 1 - 0.9 * std::fabs(pd2) / std::sqrt(point_body_vec.norm());
            if (s > 0.9)
            {
                m_point_selected_flag[i] = true;
                m_norm_vec->points[i].x = pabcd(0);
                m_norm_vec->points[i].y = pabcd(1);
                m_norm_vec->points[i].z = pabcd(2);
                m_norm_vec->points[i].intensity = pd2;
            }
        }
    }

    int effect_feat_num = 0;
    for (int i = 0; i < size; i++)
    {
        if (!m_point_selected_flag[i])
            continue;
        m_effect_cloud_lidar->points[effect_feat_num] = m_cloud_down_lidar->points[i];
        m_effect_norm_vec->points[effect_feat_num] = m_norm_vec->points[i];
        effect_feat_num++;
    }
    if (effect_feat_num < 1)
    {
        share_data.valid = false;
        std::cerr << "NO Effective Points!" << std::endl;
        return;
    }
    share_data.valid = true;
    share_data.H.setZero();
    share_data.b.setZero();

    // 构建雅可比矩阵和残差
    Eigen::MatrixXd J_all(effect_feat_num, 12);
    Eigen::VectorXd residuals(effect_feat_num);
    Eigen::Matrix<double, 1, 12> J;

    for (int i = 0; i < effect_feat_num; i++)
    {
        J.setZero();
        const PointType &laser_p = m_effect_cloud_lidar->points[i];
        const PointType &norm_p = m_effect_norm_vec->points[i];
        Eigen::Vector3d laser_p_vec(laser_p.x, laser_p.y, laser_p.z);
        Eigen::Vector3d norm_vec(norm_p.x, norm_p.y, norm_p.z);
        Eigen::Matrix<double, 1, 3> B = -norm_vec.transpose() * state.r_wi * Sophus::SO3d::hat(state.r_il * laser_p_vec + state.t_wi);
        J.block<1, 3>(0, 0) = B;
        J.block<1, 3>(0, 3) = norm_vec.transpose();
        if (m_config.esti_il)
        {
            Eigen::Matrix<double, 1, 3> C = -norm_vec.transpose() * state.r_wi * state.r_il * Sophus::SO3d::hat(laser_p_vec);
            Eigen::Matrix<double, 1, 3> D = norm_vec.transpose() * state.r_wi;
            J.block<1, 3>(0, 6) = C;
            J.block<1, 3>(0, 9) = D;
        }
        share_data.H += J.transpose() * m_config.lidar_cov_inv * J;
        share_data.b += J.transpose() * m_config.lidar_cov_inv * norm_p.intensity;

        J_all.row(i) = J;
        residuals(i) = norm_p.intensity;
    }

    // 计算信息矩阵
    Eigen::MatrixXd H_temp = J_all.transpose() * m_config.lidar_cov_inv * J_all;
    Eigen::VectorXd b_temp = J_all.transpose() * m_config.lidar_cov_inv * residuals;

    // 分析退化并调整权重
    m_is_degenerate = false;
    analyzeDegeneracy(H_temp, m_degeneracy_options);
    adjustWeightsByDegeneracy(H_temp, b_temp);

    // 更新最终的H矩阵和b向量
    share_data.H = H_temp;
    share_data.b = b_temp;
}

CloudType::Ptr LidarProcessor::transformCloud(CloudType::Ptr inp, const M3D &r, const V3D &t)
{
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block<3, 3>(0, 0) = r.cast<float>();
    transform.block<3, 1>(0, 3) = t.cast<float>();
    CloudType::Ptr ret(new CloudType);
    pcl::transformPointCloud(*inp, *ret, transform);
    return ret;
}