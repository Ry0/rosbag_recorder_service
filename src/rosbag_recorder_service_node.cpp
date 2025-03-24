#include "rclcpp/rclcpp.hpp"
#include "rosbag_recorder_service/rosbag_recorder_service.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rosbag_recorder_service::RosbagRecorderService>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
