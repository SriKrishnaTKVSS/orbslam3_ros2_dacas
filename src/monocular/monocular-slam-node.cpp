#include "monocular-slam-node.hpp"
#include <Eigen/Core>        // For Eigen matrices

#include<opencv2/core/core.hpp>

#include <thread> // For std::this_thread::sleep_for
#include <chrono> // For std::chrono::seconds


// Added for publishing the pose and orientation as ros2 message
#include "tf2_ros/transform_broadcaster.h"
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "geometry_msgs/msg/transform_stamped.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>



using std::placeholders::_1;

MonocularSlamNode::MonocularSlamNode(ORB_SLAM3::System* pSLAM)
:   Node("ORB_SLAM3_ROS2")
{
    m_SLAM = pSLAM;
    // std::cout << "slam changed" << std::endl;
    m_image_subscriber = this->create_subscription<ImageMsg>(
        "camera",
        10,
        std::bind(&MonocularSlamNode::GrabImageAndPublish, this, std::placeholders::_1)); // GrabImage
    std::cout << "slam changed" << std::endl;

    transform_pub_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("transform_topic", 10);
    transform_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud_topic", 100);
    // Timer to publish the static transform every second
    static_transform_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&MonocularSlamNode::PublishStaticTransform, this)
    );


}

MonocularSlamNode::~MonocularSlamNode()
{
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    m_SLAM->SaveTrajectoryEuRoC("CameraTrajectory.txt");

    std::this_thread::sleep_for(std::chrono::seconds(2)); 

    std::cout<< "Node destroyed"<<std::endl;
}

void MonocularSlamNode::GrabImage(const sensor_msgs::msg::Image::SharedPtr msg)
{
    // Copy the ros image message to cv::Mat.
    try
    {
        m_cvImPtr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // std::cout<<"one frame is sent"<<std::endl;
    m_SLAM->TrackMonocular(m_cvImPtr->image, Utility::StampToSec(msg->header.stamp));

    //cout<<(m_SLAM->GetTracker()->mCurrentFrame.GetPose()).matrix()<<endl;
    // float tf_matrix [4][4];
    // tf_matrix=(m_SLAM->GetTracker()->mCurrentFrame.GetPose()).matrix();
    //     // Extract rotation matrix
    // tf2::Matrix3x3 rotation_matrix(
    //         tf_matrix[0][0], tf_matrix[0][1], tf_matrix[0][2],
    //         tf_matrix[1][0], tf_matrix[1][1], tf_matrix[1][2],
    //         tf_matrix[2][0], tf_matrix[2][1], tf_matrix[2][2]);
}
void MonocularSlamNode::PublishTransform()
{

Eigen::Matrix4f tf_matrix = (m_SLAM->GetTracker()->mCurrentFrame.GetPose()).matrix();

    // Extract rotation matrix
    tf2::Matrix3x3 rotation_matrix(
            tf_matrix(0,0), tf_matrix(0,1), tf_matrix(0,2),
            tf_matrix(1,0), tf_matrix(1,1), tf_matrix(1,2),
            tf_matrix(2,0), tf_matrix(2,1), tf_matrix(2,2));

    // Convert to quaternion
    tf2::Quaternion quaternion;
    rotation_matrix.getRotation(quaternion);

    // TransformStamped
    geometry_msgs::msg::TransformStamped transform_msg;
    transform_msg.header.stamp = this->now();
    transform_msg.header.frame_id = "Camera_current_RLU";//"Camera_origin_RLU";
    transform_msg.child_frame_id = "Camera_origin_RLU";//"Camera_current_RLU";

    // Translation
    transform_msg.transform.translation.x = tf_matrix(0,3);
    transform_msg.transform.translation.y = tf_matrix(1,3);
    transform_msg.transform.translation.z = tf_matrix(2,3);

    // Rotation (quaternion)
    transform_msg.transform.rotation.x = quaternion.x();
    transform_msg.transform.rotation.y = quaternion.y();
    transform_msg.transform.rotation.z = quaternion.z();
    transform_msg.transform.rotation.w = quaternion.w();

    // Publish the message
    transform_pub_->publish(transform_msg);
    transform_broadcaster_->sendTransform(transform_msg);

    RCLCPP_INFO(this->get_logger(), "Published TransformStamped");

    // Accessing Map points
    // std::vector<ORB_SLAM3::MapPoint*> mappoints = m_SLAM->GetAllMapspoints();
    // // cout<<mappoints<<endl;
    // for (const auto& mappoint : mappoints) {
    //     if (mappoint) { // Check if the pointer is not null
    //         // Example: Access methods or members of MapPoint
    //         // std::cout << "MapPoint ID: " << mappoint->GetFoundRatio() << std::endl; 
    //         Eigen::Vector3f vec=mappoint->GetWorldPos();
    //             std::cout << "Vector: ("<< vec.x()<< ", "<< vec.y() << ", "<< vec.z() << ")" << std::endl;
    //     }
    // }
    
    
}
void MonocularSlamNode::PublishStaticTransform()
{
    geometry_msgs::msg::TransformStamped static_transform;

    static_transform.header.stamp = this->now();
    static_transform.header.frame_id = "Camera_origin_RLU";  // Parent frame
    static_transform.child_frame_id = "map";                 // Child frame

    // Identity transform (assuming "map" and "camera_origin_old" are aligned)
    static_transform.transform.translation.x = 0.0;
    static_transform.transform.translation.y = 0.0;
    static_transform.transform.translation.z = 0.0;

    static_transform.transform.rotation.x = 0.0;
    static_transform.transform.rotation.y = 0.0;
    static_transform.transform.rotation.z = 0.0;
    static_transform.transform.rotation.w = 1.0;  // No rotation

    // Publish the static transform
    transform_broadcaster_->sendTransform(static_transform);

    RCLCPP_INFO(this->get_logger(), "Published static transform: camera_origin_old -> map");
}

void MonocularSlamNode::PublishPointCloud()
{
    rclcpp::Duration interval = rclcpp::Duration::from_seconds(0.1); // Publish every 0.1 seconds
    // Locking for thread safety if needed
    std::lock_guard<std::mutex> lock(slam_mutex_);

    // Access map points directly from SLAM system
    std::vector<ORB_SLAM3::MapPoint*> mappoints = m_SLAM->GetAllMapspoints();
    size_t num_points = mappoints.size();

    // Only proceed if there are map points
    if (num_points == 0) {
        return;
    }

    // // Create and populate PointCloud2 message
    // auto point_cloud_msg = sensor_msgs::msg::PointCloud2();
    // point_cloud_msg.header.frame_id = "map";
    // point_cloud_msg.header.stamp = this->now();
    // point_cloud_msg.height = 1;
    // point_cloud_msg.width = num_points;
    // point_cloud_msg.is_dense = true;
    // point_cloud_msg.is_bigendian = false;

    // // Define fields (x, y, z) and resize the message
    // sensor_msgs::PointCloud2Modifier modifier(point_cloud_msg);
    // modifier.setPointCloud2FieldsByString(1, "xyz");
    // modifier.resize(num_points);

    // // Use iterators to populate point cloud efficiently
    // sensor_msgs::PointCloud2Iterator<float> iter_x(point_cloud_msg, "x");
    // sensor_msgs::PointCloud2Iterator<float> iter_y(point_cloud_msg, "y");
    // sensor_msgs::PointCloud2Iterator<float> iter_z(point_cloud_msg, "z");

    // // Loop over map points and fill the point cloud
    // for (const auto& point : mappoints) {
    //     if (point) {
    //         auto world_pos = point->GetWorldPos();
    //         *iter_x = world_pos.x();
    //         *iter_y = world_pos.y();
    //         *iter_z = world_pos.z();

    //         ++iter_x;
    //         ++iter_y;
    //         ++iter_z;
    //     }
    // }

    // // Publish the PointCloud2 message
    // point_cloud_pub_->publish(point_cloud_msg);
    // RCLCPP_INFO(this->get_logger(), "Published PointCloud2");

    sensor_msgs::msg::PointCloud2 point_cloud_msg;
point_cloud_msg.header.frame_id = "map";
point_cloud_msg.header.stamp = this->now();
point_cloud_msg.height = 1;
point_cloud_msg.width = num_points;
point_cloud_msg.is_dense = false;
point_cloud_msg.is_bigendian = false;

// Define the fields (x, y, z)
point_cloud_msg.fields.resize(3);
point_cloud_msg.fields[0].name = "x";
point_cloud_msg.fields[0].offset = 0;
point_cloud_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
point_cloud_msg.fields[0].count = 1;

point_cloud_msg.fields[1].name = "y";
point_cloud_msg.fields[1].offset = 4;
point_cloud_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
point_cloud_msg.fields[1].count = 1;

point_cloud_msg.fields[2].name = "z";
point_cloud_msg.fields[2].offset = 8;
point_cloud_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
point_cloud_msg.fields[2].count = 1;

point_cloud_msg.point_step = 12;  // Each point consists of 3 floats (x, y, z) -> 4 bytes * 3 = 12
point_cloud_msg.row_step = point_cloud_msg.point_step * num_points;
point_cloud_msg.data.resize(point_cloud_msg.row_step);  // Resize buffer

// Fill in the point cloud data
float* data_ptr = reinterpret_cast<float*>(point_cloud_msg.data.data());
for (const auto& point : mappoints) {
    if (point) {
        auto world_pos = point->GetWorldPos();
        *data_ptr++ = world_pos.x();
        *data_ptr++ = world_pos.y();
        *data_ptr++ = world_pos.z();
    }
}

// Publish the PointCloud2 message
point_cloud_pub_->publish(point_cloud_msg);
RCLCPP_INFO(this->get_logger(), "Published PointCloud2");

}

// Wrapper function to grab image and publish transform and point cloud
void MonocularSlamNode::GrabImageAndPublish(const ImageMsg::SharedPtr msg)
{
    // Start the threads for image grabbing, transform publishing, and point cloud publishing
    std::thread image_thread(&MonocularSlamNode::GrabImage, this, msg);
    std::thread transform_thread(&MonocularSlamNode::PublishTransform, this);
    std::thread pointcloud_thread(&MonocularSlamNode::PublishPointCloud, this);

    // Wait for all threads to finish
    image_thread.join();
    transform_thread.join();
    pointcloud_thread.join();
}