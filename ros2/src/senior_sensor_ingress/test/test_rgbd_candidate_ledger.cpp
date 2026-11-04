#include <gtest/gtest.h>
#include "senior_sensor_ingress/rgbd_candidate_ledger.hpp"

TEST(RgbdCandidateLedger, MatchesExactRawHeaderLayoutAndOriginalBodyHash)
{
  senior_sensor_ingress::RgbdCandidateLedger ledger;
  senior_motion_msgs::msg::BoundedRawImage image;
  image.capture.valid = true;
  image.capture.sequence = 1;
  image.capture.bridge_boot_id = "boot";
  image.capture.capture_generation = 1;
  image.capture.bridge_gid = "01";
  image.capture.raw_gid = "02";
  image.capture.source_epoch = "source";
  image.capture.frame_id = "camera";
  image.width = image.height = 1;
  image.step = 3;
  image.encoding = "rgb8";
  image.data = {'a', 'b', 'c'};
  ASSERT_TRUE(ledger.remember_image(0, image));
  senior_motion_msgs::msg::RawImageLayout layout;
  layout.capture = image.capture;
  layout.width = image.width;
  layout.height = image.height;
  layout.step = image.step;
  layout.encoding = image.encoding;
  layout.is_bigendian = image.is_bigendian;
  layout.byte_length = image.data.size();
  layout.sha256 = {0xba,0x78,0x16,0xbf,0x8f,0x01,0xcf,0xea,0x41,0x41,0x40,0xde,0x5d,0xae,0x22,0x23,
                   0xb0,0x03,0x61,0xa3,0x96,0x17,0x7a,0x9c,0xb4,0x10,0xff,0x61,0xf2,0x00,0x15,0xad};
  EXPECT_TRUE(ledger.match_image(0, layout));
  layout.sha256[0] ^= 1;
  EXPECT_FALSE(ledger.match_image(0, layout));
  layout.sha256[0] ^= 1;
  ++layout.capture.sequence;
  EXPECT_FALSE(ledger.match_image(0, layout));
}

TEST(RgbdCandidateLedger, SameHeaderDifferentBodyFailsClosedAndInfoMustMatchExactly)
{
  senior_sensor_ingress::RgbdCandidateLedger ledger;
  senior_motion_msgs::msg::BoundedRawImage image;
  image.capture.valid = true;
  image.capture.sequence = 1;
  image.width = image.height = 1;
  image.step = 3;
  image.encoding = "rgb8";
  image.data = {'a', 'b', 'c'};
  ASSERT_TRUE(ledger.remember_image(0, image));
  image.data[0] = 'x';
  EXPECT_FALSE(ledger.remember_image(0, image));
  ledger.clear();
  image.data[0] = 'a';
  EXPECT_FALSE(ledger.remember_image(0, image));
  ++image.capture.sequence;
  EXPECT_TRUE(ledger.remember_image(0, image));
  senior_motion_msgs::msg::BoundedRawCameraInfo info;
  info.capture.valid = true;
  info.capture.sequence = 1;
  info.width = info.height = 1;
  info.k[0] = 1.0;
  ASSERT_TRUE(ledger.remember_info(0, info));
  EXPECT_TRUE(ledger.match_info(0, info));
  info.k[0] = 2.0;
  EXPECT_FALSE(ledger.match_info(0, info));
  EXPECT_FALSE(ledger.remember_info(0, info));
  info.k[0] = 1.0;
  EXPECT_FALSE(ledger.match_info(0, info));
  ledger.clear();
  EXPECT_FALSE(ledger.remember_info(0, info));
  ++info.capture.sequence;
  EXPECT_TRUE(ledger.remember_info(0, info));
}
