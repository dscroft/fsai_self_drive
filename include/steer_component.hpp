#ifndef STEER_COMPONENT_HPP

#include <algorithm>
#include <memory>

#include <opencv2/opencv.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/dnn/dnn.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32.hpp>

namespace self_drive
{

class Steer : public rclcpp::Node
{
public:
    explicit Steer(const rclcpp::NodeOptions &options);

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_;

    cv::Mat image_;
    cv::Size inputSize_;    
    cv::dnn::Net net_;

    void image_callback(const sensor_msgs::msg::Image &msg);
};

} // namespace self_drive

#endif // STEER_COMPONENT_HPP