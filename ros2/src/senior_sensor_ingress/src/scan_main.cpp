#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "senior_sensor_ingress/scan_receipt.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<senior_sensor_ingress::ScanReceipt>());
  rclcpp::shutdown();
  return 0;
}
