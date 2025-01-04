#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include "pgos/commons.h"
#include "pgos/simple_pgo.h"
#include "utils.h"

class PGOFastLIONode : public rclcpp::Node {
private:
    // Node configurations
    Config m_pgo_config;
    std::string m_odom_topic;
    std::string m_cloud_topic;
    std::string m_world_frame;
    std::string m_body_frame;
    
    // PGO processor
    std::shared_ptr<SimplePGO> m_pgo;
    
    // Subscribers
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> m_cloud_sub;
    message_filters::Subscriber<nav_msgs::msg::Odometry> m_odom_sub;
    std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>> m_sync;

    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_corrected_odom_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_corrected_cloud_pub;
    
    void loadParams() {
        // Declare parameters
        this->declare_parameter("odom_topic", "/lio/odom");
        this->declare_parameter("cloud_topic", "/lio/cloud");
        this->declare_parameter("world_frame", "map");
        this->declare_parameter("body_frame", "body");
        this->declare_parameter("key_frame_distance", 1.0);
        this->declare_parameter("key_frame_angle_deg", 10.0);
        this->declare_parameter("loop_search_radius", 15.0);
        this->declare_parameter("loop_time_threshold", 30.0);
        this->declare_parameter("loop_score_threshold", 0.3);
        
        // Get parameters
        m_odom_topic = this->get_parameter("odom_topic").as_string();
        m_cloud_topic = this->get_parameter("cloud_topic").as_string();
        m_world_frame = this->get_parameter("world_frame").as_string();
        m_body_frame = this->get_parameter("body_frame").as_string();
        
        // Configure PGO parameters
        m_pgo_config.key_pose_delta_trans = this->get_parameter("key_frame_distance").as_double();
        m_pgo_config.key_pose_delta_deg = this->get_parameter("key_frame_angle_deg").as_double();
        m_pgo_config.loop_search_radius = this->get_parameter("loop_search_radius").as_double();
        m_pgo_config.loop_time_tresh = this->get_parameter("loop_time_threshold").as_double();
        m_pgo_config.loop_score_tresh = this->get_parameter("loop_score_threshold").as_double();
    }

    void syncCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg,
                     const nav_msgs::msg::Odometry::ConstSharedPtr& odom_msg) {
        CloudWithPose data;
        
        // Convert messages to internal format
        data.cloud.reset(new CloudType);
        pcl::fromROSMsg(*cloud_msg, *data.cloud);
        
        data.pose.setTime(cloud_msg->header.stamp.sec, cloud_msg->header.stamp.nanosec);
        data.pose.t = Eigen::Vector3d(odom_msg->pose.pose.position.x,
                                     odom_msg->pose.pose.position.y,
                                     odom_msg->pose.pose.position.z);
        data.pose.r = Eigen::Quaterniond(odom_msg->pose.pose.orientation.w,
                                        odom_msg->pose.pose.orientation.x,
                                        odom_msg->pose.pose.orientation.y,
                                        odom_msg->pose.pose.orientation.z).toRotationMatrix();

        // Process in PGO
        if(m_pgo->addKeyPose(data)) {
            m_pgo->searchForLoopPairs();
            if(m_pgo->hasLoop()) {
                m_pgo->smoothAndUpdate();
                RCLCPP_INFO(this->get_logger(), "Loop detected and optimized!");
            }
        }
        
        // Publish corrected pose and cloud
        publishCorrectedData(cloud_msg->header.stamp, data);
    }

    void publishCorrectedData(const builtin_interfaces::msg::Time& stamp, const CloudWithPose& data) {
        // Publish corrected odometry
        nav_msgs::msg::Odometry corrected_odom;
        corrected_odom.header.stamp = stamp;
        corrected_odom.header.frame_id = m_world_frame;
        corrected_odom.child_frame_id = m_body_frame;

        Eigen::Matrix3d corrected_rot = m_pgo->offsetR() * data.pose.r;
        Eigen::Vector3d corrected_pos = m_pgo->offsetR() * data.pose.t + m_pgo->offsetT();
        
        Eigen::Quaterniond q(corrected_rot);
        corrected_odom.pose.pose.orientation.w = q.w();
        corrected_odom.pose.pose.orientation.x = q.x();
        corrected_odom.pose.pose.orientation.y = q.y();
        corrected_odom.pose.pose.orientation.z = q.z();
        
        corrected_odom.pose.pose.position.x = corrected_pos.x();
        corrected_odom.pose.pose.position.y = corrected_pos.y();
        corrected_odom.pose.pose.position.z = corrected_pos.z();
        
        m_corrected_odom_pub->publish(corrected_odom);

        // Publish corrected point cloud
        sensor_msgs::msg::PointCloud2 corrected_cloud;
        CloudType::Ptr transformed_cloud(new CloudType);
        pcl::transformPointCloud(*data.cloud, *transformed_cloud, 
                               corrected_pos, q);
        pcl::toROSMsg(*transformed_cloud, corrected_cloud);
        corrected_cloud.header.stamp = stamp;
        corrected_cloud.header.frame_id = m_world_frame;
        
        m_corrected_cloud_pub->publish(corrected_cloud);
    }

public:
    PGOFastLIONode() : Node("pgo_fastlio_node") {
        // Load parameters
        loadParams();
        
        // Initialize PGO
        m_pgo = std::make_shared<SimplePGO>(m_pgo_config);
        
        // Initialize subscribers with message filters
        auto rmw_qos_profile = rmw_qos_profile_default;
        rmw_qos_profile.depth = 100;
        
        m_cloud_sub.subscribe(this, m_cloud_topic, rmw_qos_profile);
        m_odom_sub.subscribe(this, m_odom_topic, rmw_qos_profile);
        
        m_sync = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>>(
            message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>(10),
            m_cloud_sub, m_odom_sub);
            
        m_sync->registerCallback(std::bind(&PGOFastLIONode::syncCallback, this, 
                                         std::placeholders::_1,
                                         std::placeholders::_2));
        
        // Initialize publishers
        m_corrected_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>(
            "/lio/corrected_odom", 10);
        m_corrected_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/lio/corrected_cloud", 10);
            
        RCLCPP_INFO(this->get_logger(), "PGO FastLIO Node Initialized!");
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PGOFastLIONode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}