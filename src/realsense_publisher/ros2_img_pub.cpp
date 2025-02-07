#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

class RealSensePublisher : public rclcpp::Node
{
public:
    RealSensePublisher()
        : Node("realsense_publisher")
    {
        // Initialize publisher
        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera", 10);

        // Configure RealSense pipeline
        rs2::config cfg;
        cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
        pipeline_.start(cfg);

        // Start publishing loop
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),  // Approx. 30 FPS
            std::bind(&RealSensePublisher::publish_image, this));
    }

private:
    void publish_image()
    {
        // Wait for a new frame and get the color frame
        rs2::frameset frameset = pipeline_.wait_for_frames();
        rs2::video_frame color_frame = frameset.get_color_frame();

        // Convert RealSense frame to OpenCV Mat
        cv::Mat image(cv::Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);

        // Convert OpenCV Mat to ROS2 Image message
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();

        // Publish the image
        msg->header.stamp = this->get_clock()->now();
        image_publisher_->publish(*msg);
    }

    // Member variables
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rs2::pipeline pipeline_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RealSensePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
