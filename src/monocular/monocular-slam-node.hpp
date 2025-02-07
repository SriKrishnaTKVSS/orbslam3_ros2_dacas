#ifndef __MONOCULAR_SLAM_NODE_HPP__
#define __MONOCULAR_SLAM_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <Eigen/Core>        // For Eigen matrices

#include <cv_bridge/cv_bridge.h>

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

#include "utility.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"

// ROS2 message for pointcloud
#include <sensor_msgs/msg/point_cloud2.hpp>


// Include tf2_ros for transform broadcasting
#include "tf2_ros/transform_broadcaster.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>


class MonocularSlamNode : public rclcpp::Node
{
public:
    MonocularSlamNode(ORB_SLAM3::System* pSLAM);

    ~MonocularSlamNode();
    using ImageMsg = sensor_msgs::msg::Image;

        // Threaded functions
    void GrabImage(const sensor_msgs::msg::Image::SharedPtr msg);
    void PublishTransform();
    void PublishPointCloud();
        // Wrapper function to call all threaded tasks
    void GrabImageAndPublish(const sensor_msgs::msg::Image::SharedPtr msg);

private:
    

    // void GrabImage(const sensor_msgs::msg::Image::SharedPtr msg);

    ORB_SLAM3::System* m_SLAM;

    cv_bridge::CvImagePtr m_cvImPtr;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_image_subscriber;

    // Member variable for publisher
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr transform_pub_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;  // Transform broadcaster
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
    std::mutex slam_mutex_;  // Mutex for thread safety

    void PublishStaticTransform();  // New function declaration

    rclcpp::TimerBase::SharedPtr static_transform_timer_; // Timer for periodic transform publishing
};

#endif
