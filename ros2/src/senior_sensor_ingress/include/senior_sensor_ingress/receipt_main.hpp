#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace senior_sensor_ingress
{
template<typename Receipt>
int run_receipt(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const auto context = rclcpp::contexts::get_global_default_context();
  struct Shutdown
  {
    rclcpp::Context::SharedPtr context;
    ~Shutdown() {rclcpp::shutdown(context);}
  } shutdown{context};

  try {
    rclcpp::spin(std::make_shared<Receipt>());
  } catch (const rclcpp::exceptions::RCLError &) {
    // //Compatible: a Humble graph query may race native context shutdown.
    if (context->is_valid()) {
      throw;
    }
  }
  return 0;
}
}  // namespace senior_sensor_ingress
