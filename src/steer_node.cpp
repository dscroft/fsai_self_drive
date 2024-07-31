#include <rclcpp/rclcpp.hpp>

#include <memory>

#include "steer_component.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);
  rclcpp::spin(std::make_shared<self_drive::Steer>(options));
  rclcpp::shutdown();
  return 0;
}