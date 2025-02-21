#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class TransformBroadcasterNode : public rclcpp::Node {
public:
    TransformBroadcasterNode() : Node("tf_publish_RLU2MOCAP") {
        broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&TransformBroadcasterNode::broadcast_transform, this));
    }

private:
    void broadcast_transform() {
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = this->get_clock()->now();
        transformStamped.header.frame_id ="Camera_origin_RLU" ;  // Parent frame
        transformStamped.child_frame_id = "mocap_at_camera_origin";   // Child frame
        transformStamped.transform.translation.x = 0.0;
        transformStamped.transform.translation.y = 0.0;
        transformStamped.transform.translation.z = 0.0;
        transformStamped.transform.rotation.x = 0.5;//-0.5;//-0.5 rest 0
        transformStamped.transform.rotation.y = -0.5;//0.5;
        transformStamped.transform.rotation.z = 0.5;//-0.5;
        transformStamped.transform.rotation.w = 0.5;

        broadcaster_->sendTransform(transformStamped);
    }

    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TransformBroadcasterNode>());
    rclcpp::shutdown();
    return 0;
}
