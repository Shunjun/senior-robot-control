#include "senior_sensor_ingress/receipt_main.hpp"
#include "senior_sensor_ingress/static_tf_receipt.hpp"

int main(int argc, char ** argv)
{
  return senior_sensor_ingress::run_receipt<senior_sensor_ingress::StaticTfReceipt>(argc, argv);
}
