#include "steer_component.hpp"

#include <cv_bridge/cv_bridge.h>

#include <functional>
#include <string>

namespace self_drive
{

Steer::Steer(const rclcpp::NodeOptions &options) : Node("Steer", options)
{
    /* === parameter handling === */
    const std::string modelPath = 
        this->declare_parameter<std::string>("model_path", "model.onnx");
    const bool cpu = 
        this->declare_parameter<bool>("cpu", false);
    const int width = 
        this->declare_parameter<int>("input_width", 100);
    const int height = 
        this->declare_parameter<int>("input_height", 100);

    this->inputSize_ = cv::Size(width, height);

    RCLCPP_INFO_STREAM(this->get_logger(), "model_path: " << modelPath);
    RCLCPP_INFO_STREAM(this->get_logger(), "cpu: " << std::boolalpha << cpu);
    RCLCPP_INFO_STREAM(this->get_logger(), "input_width: " << width);
    RCLCPP_INFO_STREAM(this->get_logger(), "input_height: " << height);

    /* === configure and load dnn === */
    try
    {
        this->net_ = cv::dnn::readNetFromONNX( modelPath );
    }
    catch(const std::exception& e)
    {
        RCLCPP_FATAL_STREAM(this->get_logger(), "Failed to load model: " << e.what());
        rclcpp::shutdown();
    }

    if( cpu )
    {
        this->net_.setPreferableBackend(cv::dnn::Backend::DNN_BACKEND_OPENCV);
        this->net_.setPreferableTarget(cv::dnn::Target::DNN_TARGET_CPU);
    }
    else
    {
        this->net_.setPreferableBackend(cv::dnn::Backend::DNN_BACKEND_CUDA);
        this->net_.setPreferableTarget(cv::dnn::Target::DNN_TARGET_CUDA);
    }

    /* === setup subscription and publisher === */
    this->sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "image", 10, std::bind(&Steer::image_callback, this, std::placeholders::_1));

    this->pub_ = this->create_publisher<std_msgs::msg::Float32>(
        "steer_angle", 10);
}

void Steer::image_callback(const sensor_msgs::msg::Image &msg)
{
    RCLCPP_INFO_STREAM(this->get_logger(), "Received image " << msg.encoding);

    try
    {
        // only tested for BGR8 and BGRA8
        this->image_ = cv_bridge::toCvCopy(msg, msg.encoding)->image;
    }
    catch( cv_bridge::Exception& e )
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // prep image for processing
    cv::Mat gray;
    cv::cvtColor(this->image_, gray, cv::COLOR_BGR2GRAY);

    cv::Mat blob = cv::dnn::blobFromImage(gray, 1, cv::Size(100,100));

    // process image
    this->net_.setInput(blob);
    cv::Mat output = this->net_.forward();

    // get result
    const float steerAngle = output.at<float>(0, 0);

    RCLCPP_INFO_STREAM(this->get_logger(), "Output: " << output << " " << steerAngle );

    // publish result
    std_msgs::msg::Float32 msgOut;
    msgOut.data = steerAngle;
    this->pub_->publish(msgOut);
}

} // namespace cnn_ros

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(self_drive::Steer)
