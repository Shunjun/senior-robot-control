#include "senior_sensor_ingress/receipt_main.hpp"
#include "senior_sensor_ingress/rgbd_candidate_receipt.hpp"
int main(int argc, char ** argv)
{
  return senior_sensor_ingress::run_receipt<senior_sensor_ingress::RgbdCandidateReceipt>(argc, argv);
}
