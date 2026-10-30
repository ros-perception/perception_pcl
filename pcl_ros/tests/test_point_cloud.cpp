// Copyright (c) 2026, Open Source Robotics Foundation, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <gtest/gtest.h>

#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "pcl_ros/point_cloud.hpp"
#include "pcl/point_types.h"

struct TestMemOStream
{
  std::vector<uint8_t> buffer_;

  template<typename T>
  void next(const T & val)
  {
    const uint8_t * p = reinterpret_cast<const uint8_t *>(&val);
    buffer_.insert(buffer_.end(), p, p + sizeof(T));
  }

  void next(const std::string & str)
  {
    uint32_t len = static_cast<uint32_t>(str.size());
    next(len);
    buffer_.insert(buffer_.end(), str.begin(), str.end());
  }

  void next(const pcl::PCLHeader & h)
  {
    next(static_cast<uint32_t>(h.stamp >> 32));
    next(static_cast<uint32_t>(h.stamp & 0xFFFFFFFFULL));
    next(h.frame_id);
  }

  uint8_t * advance(size_t len)
  {
    size_t old_size = buffer_.size();
    buffer_.resize(old_size + len);
    return buffer_.data() + old_size;
  }
};

struct TestMemIStream
{
  const uint8_t * data_;
  size_t length_;
  size_t offset_{0};

  TestMemIStream(const uint8_t * data, size_t length)
  : data_(data), length_(length) {}

  template<typename T>
  void next(T & val)
  {
    if (offset_ + sizeof(T) > length_) {
      throw std::runtime_error("Stream overrun");
    }
    std::memcpy(&val, data_ + offset_, sizeof(T));
    offset_ += sizeof(T);
  }

  void next(std::string & str)
  {
    uint32_t len;
    next(len);
    if (offset_ + len > length_) {
      throw std::runtime_error("Stream overrun");
    }
    str.assign(reinterpret_cast<const char *>(data_ + offset_), len);
    offset_ += len;
  }

  void next(std_msgs::msg::Header & h)
  {
    next(h.stamp.sec);
    next(h.stamp.nanosec);
    next(h.frame_id);
  }

  void next(std::vector<sensor_msgs::msg::PointField> & fields)
  {
    uint32_t count;
    next(count);
    fields.resize(count);
    for (uint32_t i = 0; i < count; ++i) {
      next(fields[i].name);
      next(fields[i].offset);
      next(fields[i].datatype);
      next(fields[i].count);
    }
  }

  const uint8_t * advance(size_t len)
  {
    if (offset_ + len > length_) {
      throw std::runtime_error("Stream overrun");
    }
    const uint8_t * ptr = data_ + offset_;
    offset_ += len;
    return ptr;
  }
};

TEST(TestPointCloudSerializer, round_trip)
{
  pcl::PointCloud<pcl::PointXYZ> cloud_out;
  cloud_out.header.frame_id = "test_frame";
  cloud_out.width = 2;
  cloud_out.height = 1;
  cloud_out.is_dense = true;

  pcl::PointXYZ p1{1.0f, 2.0f, 3.0f};
  pcl::PointXYZ p2{4.0f, 5.0f, 6.0f};
  cloud_out.points.push_back(p1);
  cloud_out.points.push_back(p2);

  TestMemOStream ostream;
  ros::serialization::Serializer<pcl::PointCloud<pcl::PointXYZ>>::write(ostream, cloud_out);

  TestMemIStream istream(ostream.buffer_.data(), ostream.buffer_.size());
  pcl::PointCloud<pcl::PointXYZ> cloud_in;
  ros::serialization::Serializer<pcl::PointCloud<pcl::PointXYZ>>::read(istream, cloud_in);

  EXPECT_EQ(cloud_in.header.frame_id, "test_frame");
  EXPECT_EQ(cloud_in.width, 2u);
  EXPECT_EQ(cloud_in.height, 1u);
  ASSERT_EQ(cloud_in.points.size(), 2u);
  EXPECT_FLOAT_EQ(cloud_in.points[0].x, 1.0f);
  EXPECT_FLOAT_EQ(cloud_in.points[0].y, 2.0f);
  EXPECT_FLOAT_EQ(cloud_in.points[0].z, 3.0f);
  EXPECT_FLOAT_EQ(cloud_in.points[1].x, 4.0f);
  EXPECT_FLOAT_EQ(cloud_in.points[1].y, 5.0f);
  EXPECT_FLOAT_EQ(cloud_in.points[1].z, 6.0f);
}

TEST(TestPointCloudSerializer, overflow_reproducer_issue_39)
{
  // Malicious / corrupt message:
  // width: 0xffffffff, height: 0xffffffff, point_step: 1, row_step: 0xffffffff, data_size: 1
  TestMemOStream ostream;

  // Header: stamp (sec=0, nanosec=0), frame_id="frame"
  ostream.next(static_cast<uint32_t>(0));
  ostream.next(static_cast<uint32_t>(0));
  ostream.next(std::string("frame"));

  // Height and Width
  ostream.next(static_cast<uint32_t>(0xffffffff));
  ostream.next(static_cast<uint32_t>(0xffffffff));

  // Fields: 2 fields (x, y)
  ostream.next(static_cast<uint32_t>(2));
  ostream.next(std::string("x"));
  ostream.next(static_cast<uint32_t>(0));
  ostream.next(static_cast<uint8_t>(7));  // FLOAT32
  ostream.next(static_cast<uint32_t>(1));
  ostream.next(std::string("y"));
  ostream.next(static_cast<uint32_t>(4));
  ostream.next(static_cast<uint8_t>(7));  // FLOAT32
  ostream.next(static_cast<uint32_t>(1));

  // is_bigendian
  ostream.next(static_cast<uint8_t>(0));

  // point_step and row_step
  ostream.next(static_cast<uint32_t>(1));
  ostream.next(static_cast<uint32_t>(0xffffffff));

  // data_size and 1 byte of payload
  ostream.next(static_cast<uint32_t>(1));
  uint8_t * payload = ostream.advance(1);
  payload[0] = 0;

  // is_dense
  ostream.next(static_cast<uint8_t>(1));

  TestMemIStream istream(ostream.buffer_.data(), ostream.buffer_.size());
  pcl::PointCloud<pcl::PointXY> cloud_in;

  // Deserializing this corrupt message must throw an exception and NOT segfault
  EXPECT_THROW(
    ros::serialization::Serializer<pcl::PointCloud<pcl::PointXY>>::read(istream, cloud_in),
    std::runtime_error
  );
}

TEST(TestPointCloudSerializer, row_step_smaller_than_min_row_step)
{
  TestMemOStream ostream;
  // Header
  ostream.next(static_cast<uint32_t>(0));
  ostream.next(static_cast<uint32_t>(0));
  ostream.next(std::string("frame"));

  // Height=1, Width=10
  ostream.next(static_cast<uint32_t>(1));
  ostream.next(static_cast<uint32_t>(10));

  // Fields: 1 field
  ostream.next(static_cast<uint32_t>(1));
  ostream.next(std::string("x"));
  ostream.next(static_cast<uint32_t>(0));
  ostream.next(static_cast<uint8_t>(7));
  ostream.next(static_cast<uint32_t>(1));

  // is_bigendian
  ostream.next(static_cast<uint8_t>(0));

  // point_step=16, row_step=10 (invalid, smaller than width * point_step = 160)
  ostream.next(static_cast<uint32_t>(16));
  ostream.next(static_cast<uint32_t>(10));

  // data_size=10
  ostream.next(static_cast<uint32_t>(10));
  ostream.advance(10);
  ostream.next(static_cast<uint8_t>(1));

  TestMemIStream istream(ostream.buffer_.data(), ostream.buffer_.size());
  pcl::PointCloud<pcl::PointXYZ> cloud_in;

  EXPECT_THROW(
    ros::serialization::Serializer<pcl::PointCloud<pcl::PointXYZ>>::read(istream, cloud_in),
    std::runtime_error
  );
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
