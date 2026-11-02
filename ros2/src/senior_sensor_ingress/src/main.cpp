#include <memory>

#include <rclcpp/rclcpp.hpp>
#include "senior_sensor_ingress/raw_receipt.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<senior_sensor_ingress::RawReceipt>());
  rclcpp::shutdown();
  return 0;
}
