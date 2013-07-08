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

} // namespace pcl_conversions

#endif