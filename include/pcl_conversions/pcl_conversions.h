/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2013, Open Source Robotics Foundation, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Open Source Robotics Foundation, Inc. nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef PCL_CONVERSIONS_H__
#define PCL_CONVERSIONS_H__

#include <vector>

#include <ros/ros.h>

#include <pcl/conversions.h>

#include <pcl/PCLHeader.h>
#include <std_msgs/Header.h>

#include <pcl/PCLImage.h>
#include <sensor_msgs/Image.h>

#include <pcl/PCLPointField.h>
#include <sensor_msgs/PointField.h>

#include <pcl/PCLPointCloud2.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/PointIndices.h>
#include <pcl_msgs/PointIndices.h>

#include <pcl/io/pcd_io.h>

#include <Eigen/StdVector>
#include <Eigen/Geometry>

namespace pcl_conversions {

/** PCLHeader <=> Header **/

void fromPCL(const pcl::PCLHeader &pcl_header, std_msgs::Header &header)
{
    header.stamp.fromNSec(pcl_header.stamp * 1e3);  // Convert from us to ns
    header.seq = pcl_header.seq;
    header.frame_id = pcl_header.frame_id;
}

void toPCL(const std_msgs::Header &header, pcl::PCLHeader &pcl_header)
{
    pcl_header.stamp = header.stamp.toNSec() / 1e3;  // Convert from ns to us
    pcl_header.seq = header.seq;
    pcl_header.frame_id = header.frame_id;
}

std_msgs::Header fromPCL(const pcl::PCLHeader &pcl_header)
{
    std_msgs::Header header;
    fromPCL(pcl_header, header);
    return header;
}

pcl::PCLHeader toPCL(const std_msgs::Header &header)
{
    pcl::PCLHeader pcl_header;
    toPCL(header, pcl_header);
    return pcl_header;
}

/** PCLImage <=> Image **/

void fromPCL(const pcl::PCLImage &pcl_image, sensor_msgs::Image &image)
{
    fromPCL(pcl_image.header, image.header);
    image.height = pcl_image.height;
    image.width = pcl_image.width;
    image.encoding = pcl_image.encoding;
    image.is_bigendian = pcl_image.is_bigendian;
    image.step = pcl_image.step;
    image.data = pcl_image.data;
}

void toPCL(const sensor_msgs::Image &image, pcl::PCLImage &pcl_image)
{
    toPCL(image.header, pcl_image.header);
    pcl_image.height = image.height;
    pcl_image.width = image.width;
    pcl_image.encoding = image.encoding;
    pcl_image.is_bigendian = image.is_bigendian;
    pcl_image.step = image.step;
    pcl_image.data = image.data;
}

/** PCLPointCloud2 <=> PointCloud2 **/

void fromPCL(const pcl::PCLPointField &pcl_pf, sensor_msgs::PointField &pf)
{
    pf.name = pcl_pf.name;
    pf.offset = pcl_pf.offset;
    pf.datatype = pcl_pf.datatype;
    pf.count = pcl_pf.count;
}

void fromPCL(const std::vector<pcl::PCLPointField> &pcl_pfs, std::vector<sensor_msgs::PointField> &pfs)
{
    pfs.resize(pcl_pfs.size());
    std::vector<pcl::PCLPointField>::const_iterator it = pcl_pfs.begin();
    int i = 0;
    for(; it != pcl_pfs.end(); ++it, ++i) {
        fromPCL(*(it), pfs[i]);
    }
}

void toPCL(const sensor_msgs::PointField &pf, pcl::PCLPointField &pcl_pf)
{
    pcl_pf.name = pf.name;
    pcl_pf.offset = pf.offset;
    pcl_pf.datatype = pf.datatype;
    pcl_pf.count = pf.count;
}

void toPCL(const std::vector<sensor_msgs::PointField> &pfs, std::vector<pcl::PCLPointField> &pcl_pfs)
{
    pcl_pfs.resize(pfs.size());
    std::vector<sensor_msgs::PointField>::const_iterator it = pfs.begin();
    int i = 0;
    for(; it != pfs.end(); ++it, ++i) {
        toPCL(*(it), pcl_pfs[i]);
    }
}

/** PCLPointCloud2 <=> PointCloud2 **/

void fromPCL(const pcl::PCLPointCloud2 &pcl_pc2, sensor_msgs::PointCloud2 &pc2)
{
    fromPCL(pcl_pc2.header, pc2.header);
    pc2.height = pcl_pc2.height;
    pc2.width = pcl_pc2.width;
    fromPCL(pcl_pc2.fields, pc2.fields);
    pc2.is_bigendian = pcl_pc2.is_bigendian;
    pc2.point_step = pcl_pc2.point_step;
    pc2.row_step = pcl_pc2.row_step;
    pc2.data = pcl_pc2.data;
    pc2.is_dense = pcl_pc2.is_dense;
}

void toPCL(const sensor_msgs::PointCloud2 &pc2, pcl::PCLPointCloud2 &pcl_pc2)
{
    toPCL(pc2.header, pcl_pc2.header);
    pcl_pc2.height = pc2.height;
    pcl_pc2.width = pc2.width;
    toPCL(pc2.fields, pcl_pc2.fields);
    pcl_pc2.is_bigendian = pc2.is_bigendian;
    pcl_pc2.point_step = pc2.point_step;
    pcl_pc2.row_step = pc2.row_step;
    pcl_pc2.data = pc2.data;
    pcl_pc2.is_dense = pc2.is_dense;
}

/** pcl::PointIndices <=> pcl_msgs::PointIndices **/
void fromPCL(const pcl::PointIndices &pcl_pi, pcl_msgs::PointIndices &pi)
{
  fromPCL(pcl_pi.header, pi.header);
  pi.indices = pcl_pi.indices;
}

void toPCL(const pcl_msgs::PointIndices &pi, pcl::PointIndices &pcl_pi)
{
  toPCL(pi.header, pcl_pi.header);
  pcl_pi.indices = pi.indices;
}

} // namespace pcl_conversions

namespace pcl {

/** Overload pcl::getFieldIndex **/
inline int getFieldIndex(const sensor_msgs::PointCloud2 &cloud, const std::string &field_name)
{
    // Get the index we need
    for (size_t d = 0; d < cloud.fields.size(); ++d) {
        if (cloud.fields[d].name == field_name) {
            return (static_cast<int>(d));
        }
    }
    return (-1);

}

/** Overload pcl::getFieldsList **/
inline std::string getFieldsList(const sensor_msgs::PointCloud2 &cloud)
{
    std::string result;
    for (size_t i = 0; i < cloud.fields.size () - 1; ++i) {
        result += cloud.fields[i].name + " ";
    }
    result += cloud.fields[cloud.fields.size () - 1].name;
    return (result);
}

/** Provide pcl::toROSMsg **/
void toROSMsg(const sensor_msgs::PointCloud2 &cloud, sensor_msgs::Image &image)
{
    pcl::PCLPointCloud2 pcl_cloud;
    pcl_conversions::toPCL(cloud, pcl_cloud);
    pcl::PCLImage pcl_image;
    pcl::toPCLPointCloud2(pcl_cloud, pcl_image);
    pcl_conversions::fromPCL(pcl_image, image);
}

/** Overload pcl::createMapping **/
template<typename PointT>
void createMapping(const std::vector<sensor_msgs::PointField>& msg_fields, MsgFieldMap& field_map)
{
    std::vector<pcl::PCLPointField> pcl_msg_fields;
    pcl_conversions::toPCL(msg_fields, pcl_msg_fields);
    return createMapping<PointT>(pcl_msg_fields, field_map);
}

namespace io {

/** Overload pcl::io::savePCDFile **/
inline int 
savePCDFile(const std::string &file_name, const sensor_msgs::PointCloud2 &cloud,
            const Eigen::Vector4f &origin = Eigen::Vector4f::Zero (), 
            const Eigen::Quaternionf &orientation = Eigen::Quaternionf::Identity (),
            const bool binary_mode = false)
{
    pcl::PCLPointCloud2 pcl_cloud;
    pcl_conversions::toPCL(cloud, pcl_cloud);
    return pcl::io::savePCDFile(file_name, pcl_cloud, origin, orientation, binary_mode);
}

/** Overload pcl::io::loadPCDFile **/
inline int loadPCDFile(const std::string &file_name, sensor_msgs::PointCloud2 &cloud)
{
    pcl::PCLPointCloud2 pcl_cloud;
    int ret = pcl::io::loadPCDFile(file_name, pcl_cloud);
    pcl_conversions::fromPCL(pcl_cloud, cloud);
    return ret;
}

} // namespace io

} // namespace pcl

/*
 * Provide a custom serialization for pcl::PCLPointCloud2
 */

namespace ros 
{
  // In ROS 1.3.1+, we can specialize the functor used to create PointCloud<T> objects
  // on the subscriber side. This allows us to generate the mapping between message
  // data and object fields only once and reuse it.
#if ROS_VERSION_MINIMUM(1, 3, 1)
  template<>
  struct DefaultMessageCreator<pcl::PCLPointCloud2>
  { 
    boost::shared_ptr<pcl::PCLPointCloud2> operator() ()
    {
      boost::shared_ptr<pcl::PCLPointCloud2> msg(new pcl::PCLPointCloud2());
      return msg;
    }
  };
#endif

  namespace message_traits 
  {
    template<>
    struct MD5Sum<pcl::PCLPointCloud2>
    {
      static const char* value() { return MD5Sum<sensor_msgs::PointCloud2>::value(); }
      static const char* value(const pcl::PCLPointCloud2&) { return value(); }

      static const uint64_t static_value1 = MD5Sum<sensor_msgs::PointCloud2>::static_value1;
      static const uint64_t static_value2 = MD5Sum<sensor_msgs::PointCloud2>::static_value2;
      
      // If the definition of sensor_msgs/PointCloud2 changes, we'll get a compile error here.
      ROS_STATIC_ASSERT(static_value1 == 0x1158d486dd51d683ULL);
      ROS_STATIC_ASSERT(static_value2 == 0xce2f1be655c3c181ULL);
    };

    template<>
    struct DataType<pcl::PCLPointCloud2>
    {
      static const char* value() { return DataType<sensor_msgs::PointCloud2>::value(); }
      static const char* value(const pcl::PCLPointCloud2&) { return value(); }
    };

    template<>
    struct Definition<pcl::PCLPointCloud2>
    {
      static const char* value() { return Definition<sensor_msgs::PointCloud2>::value(); }
      static const char* value(const pcl::PCLPointCloud2&) { return value(); }
    };

    template<> struct HasHeader<pcl::PCLPointCloud2> : TrueType {};
  } // namespace ros::message_traits

  namespace serialization 
  {
    template<>
    struct Serializer<pcl::PCLPointCloud2>
    {
      template<typename Stream>
      inline static void write(Stream& stream, const pcl::PCLPointCloud2& m)
      {
        std_msgs::Header header;
        pcl_conversions::fromPCL(m.header, header);
        stream.next(header);
        stream.next(m.height);
        stream.next(m.width);
        std::vector<sensor_msgs::PointField> pfs;
        pcl_conversions::fromPCL(m.fields, pfs);
        stream.next(pfs);
        stream.next(m.is_bigendian);
        stream.next(m.point_step);
        stream.next(m.row_step);
        stream.next(m.data);
        stream.next(m.is_dense);
      }

      template<typename Stream>
      inline static void read(Stream& stream, pcl::PCLPointCloud2& m)
      {
        std_msgs::Header header;
        stream.next(header);
        pcl_conversions::toPCL(header, m.header);
        stream.next(m.height);
        stream.next(m.width);
        std::vector<sensor_msgs::PointField> pfs;
        stream.next(pfs);
        pcl_conversions::toPCL(pfs, m.fields);
        stream.next(m.fields);
        stream.next(m.is_bigendian);
        stream.next(m.point_step);
        stream.next(m.row_step);
        stream.next(m.data);
        stream.next(m.is_dense);
      }

      inline static uint32_t serializedLength(const pcl::PCLPointCloud2& m)
      {
        uint32_t length = 0;

        std_msgs::Header header;
        pcl_conversions::fromPCL(m.header, header);
        length += serializationLength(header);
        length += 4; // height
        length += 4; // width
        std::vector<sensor_msgs::PointField> pfs;
        pcl_conversions::fromPCL(m.fields, pfs);
        length += serializationLength(pfs); // fields
        length += 1; // is_bigendian
        length += 4; // point_step
        length += 4; // row_step
        length += 4; // data's size
        length += m.data.size() * sizeof(pcl::uint8_t);
        length += 1; // is_dense

        return length;
      }
    };

    template<>
    struct Serializer<pcl::PCLHeader>
    {
      template<typename Stream>
      inline static void write(Stream& stream, const pcl::PCLHeader& m)
      {
        std_msgs::Header header;
        pcl_conversions::fromPCL(m, header);
        stream.next(header);
      }

      template<typename Stream>
      inline static void read(Stream& stream, pcl::PCLHeader& m)
      {
        std_msgs::Header header;
        stream.next(header);
        pcl_conversions::toPCL(header, m);
      }

      inline static uint32_t serializedLength(const pcl::PCLHeader& m)
      {
        uint32_t length = 0;

        std_msgs::Header header;
        pcl_conversions::fromPCL(m, header);
        length += serializationLength(header);

        return length;
      }
    };
  } // namespace ros::serialization

} // namespace ros


#endif