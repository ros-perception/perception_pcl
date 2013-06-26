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

#include <vector>

#include <boost/shared_ptr.hpp>

#include <pcl_std_msgs/PCLHeader.h>
#include <std_msgs/Header.h>

#include <pcl_sensor_msgs/PCLImage.h>
#include <sensor_msgs/Image.h>

#include <pcl_sensor_msgs/PCLPointField.h>
#include <sensor_msgs/PointField.h>

#include <pcl_sensor_msgs/PCLPointCloud2.h>
#include <sensor_msgs/PointCloud2.h>

namespace pcl_conversions {

/** PCLHeader <=> Header **/

void fromPCLHeaderToHeader(const pcl_std_msgs::PCLHeader &pcl_header, std_msgs::Header &header)
{
    header.stamp.fromNSec(pcl_header.stamp);
    header.seq = pcl_header.seq;
    header.frame_id = pcl_header.frame_id;
}

void fromHeaderToPCLHeader(const std_msgs::Header &header, pcl_std_msgs::PCLHeader &pcl_header)
{
    pcl_header.stamp = header.stamp.toNSec();
    pcl_header.seq = header.seq;
    pcl_header.frame_id = header.frame_id;
}

/** PCLImage <=> Image **/

void fromPCLImageToImage(const pcl_sensor_msgs::PCLImage &pcl_image, sensor_msgs::Image &image)
{
    fromPCLHeaderToHeader(pcl_image.header, image.header);
    image.height = pcl_image.height;
    image.width = pcl_image.width;
    image.encoding = pcl_image.encoding;
    image.is_bigendian = pcl_image.is_bigendian;
    image.step = pcl_image.step;
    image.data = pcl_image.data;
}

class ConvertedImage
{
public:
  ConvertedImage(boost::shared_ptr<pcl_sensor_msgs::PCLImage> pcl_image_ptr)
  : image(), _pcl_image_ptr(pcl_image_ptr)
  {
    fromPCLHeaderToHeader(pcl_image_ptr->header, image.header);
    image.height = pcl_image_ptr->height;
    image.width = pcl_image_ptr->width;
    image.encoding = pcl_image_ptr->encoding;
    image.is_bigendian = pcl_image_ptr->is_bigendian;
    image.step = pcl_image_ptr->step;
  }

  sensor_msgs::Image image;
private:
  boost::shared_ptr<pcl_sensor_msgs::PCLImage> _pcl_image_ptr;
};

void fromImageToPCLImage(const sensor_msgs::Image &image, pcl_sensor_msgs::PCLImage &pcl_image)
{
    fromHeaderToPCLHeader(image.header, pcl_image.header);
    pcl_image.height = image.height;
    pcl_image.width = image.width;
    pcl_image.encoding = image.encoding;
    pcl_image.is_bigendian = image.is_bigendian;
    pcl_image.step = image.step;
    pcl_image.data = image.data;
}

/** PCLPointCloud2 <=> PointCloud2 **/

void fromPCLPointFieldToPointField(const pcl_sensor_msgs::PCLPointField &pcl_pf, sensor_msgs::PointField &pf)
{
    pf.name = pcl_pf.name;
    pf.offset = pcl_pf.offset;
    pf.datatype = pcl_pf.datatype;
    pf.count = pcl_pf.count;
}

void fromPointFieldToPCLPointField(const sensor_msgs::PointField &pf, pcl_sensor_msgs::PCLPointField &pcl_pf)
{
    pcl_pf.name = pf.name;
    pcl_pf.offset = pf.offset;
    pcl_pf.datatype = pf.datatype;
    pcl_pf.count = pf.count;
}

/** PCLPointCloud2 <=> PointCloud2 **/

void fromPCLPointCloud2ToPointCloud2(const pcl_sensor_msgs::PCLPointCloud2 &pcl_pc2, sensor_msgs::PointCloud2 &pc2)
{
    fromPCLHeaderToHeader(pcl_pc2.header, pc2.header);
    pc2.height = pcl_pc2.height;
    pc2.width = pcl_pc2.width;
    pc2.fields.resize(pcl_pc2.fields.size());
    std::vector<pcl_sensor_msgs::PCLPointField>::const_iterator it = pcl_pc2.fields.begin();
    int i = 0;
    for(; it != pcl_pc2.fields.end(); ++it, ++i) {
        fromPCLPointFieldToPointField(*(it), pc2.fields[i]);
    }
    pc2.is_bigendian = pcl_pc2.is_bigendian;
    pc2.point_step = pcl_pc2.point_step;
    pc2.row_step = pcl_pc2.row_step;
    pc2.data = pcl_pc2.data;
    pc2.is_dense = pcl_pc2.is_dense;
}

void fromPointCloud2ToPCLPointCloud2(const sensor_msgs::PointCloud2 &pc2, pcl_sensor_msgs::PCLPointCloud2 &pcl_pc2)
{
    fromHeaderToPCLHeader(pc2.header, pcl_pc2.header);
    pcl_pc2.height = pc2.height;
    pcl_pc2.width = pc2.width;
    pcl_pc2.fields.resize(pc2.fields.size());
    std::vector<sensor_msgs::PointField>::const_iterator it = pc2.fields.begin();
    int i = 0;
    for(; it != pc2.fields.end(); ++it, ++i) {
        fromPointFieldToPCLPointField(*(it), pcl_pc2.fields[i]);
    }
    pcl_pc2.is_bigendian = pc2.is_bigendian;
    pcl_pc2.point_step = pc2.point_step;
    pcl_pc2.row_step = pc2.row_step;
    pcl_pc2.data = pc2.data;
    pcl_pc2.is_dense = pc2.is_dense;
}

} // namespace pcl_conversions
