#include "senior_sensor_ingress/scan_receipt.hpp"
#include "senior_sensor_ingress/receipt_main.hpp"

int main(int argc, char ** argv)
{
  return senior_sensor_ingress::run_receipt<senior_sensor_ingress::ScanReceipt>(argc, argv);
}
