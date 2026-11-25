## Overview

This package provides interfaces and tools for bridging a running ROS
system to the Point Cloud Library. These include ROS nodelets, nodes,
and C++ interfaces.

## ROS nodelets

`pcl_ros` includes several
[PCL filters](http://www.ros.org/wiki/pcl_ros/Filters) packaged as ROS
nodelets. These links provide details for using those interfaces:

- [Extract_Indices](http://www.ros.org/wiki/pcl_ros/Tutorials/filters#Extract_Indices)
- [PassThrough](http://www.ros.org/wiki/pcl_ros/Tutorials/filters#PassThrough)
- [ProjectInliers](http://www.ros.org/wiki/pcl_ros/Tutorials/filters#ProjectInliers)
- [RadiusOutlierRemoval](http://www.ros.org/wiki/pcl_ros/Tutorials/filters#RadiusOutlierRemoval)
- [StatisticalOutlierRemoval](http://www.ros.org/wiki/pcl_ros/Tutorials/filters#StatisticalOutlierRemoval)
- [VoxelGrid](http://www.ros.org/wiki/pcl_ros/Tutorials/filters#VoxelGrid)

## ROS C++ interface

`pcl_ros` extends the ROS
[C++ client library](https://index.ros.org/p/roscpp) to support message
passing with [PCL](pcl) native data types. Simply add the following
include to your ROS node source code: 

```
#include <pcl_ros/point_cloud.h>
```



This header allows you to publish and subscribe
[pcl::PointCloud<T>](http://www.ros.org/doc/api/pcl/html/classpcl_1_1PointCloud.html)
objects as ROS messages. These appear to ROS as
[sensor_msgs/msg/PointCloud2](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/PointCloud2.html)
messages, offering seamless interoperability with non-PCL-using ROS
nodes. For example, you may publish a `pcl::PointCloud<T>` in one of
your nodes and visualize it in [rviz](https://index.ros.org/p/rviz)
using a [Point Cloud2 display](https://index.ros.org/p/rviz). It works
by hooking into the
[roscpp serialization](https://index.ros.org/p/roscpp) infrastructure.

The old format
[sensor_msgs/msg/PointCloud](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/PointCloud.html)
is not supported in PCL.

### Publishing point clouds

You may publish PCL point clouds using the standard `ros::Publisher`:



```cplusplus
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

int main(int argc, char** argv)
{
  ros::init (argc, argv, "pub_pcl");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<PointCloud> ("points2", 1);

  PointCloud::Ptr msg (new PointCloud);
  msg->header.frame_id = "some_tf_frame";
  msg->height = msg->width = 1;
  msg->points.push_back (pcl::PointXYZ(1.0, 2.0, 3.0));

  ros::Rate loop_rate(4);
  while (nh.ok())
  {
    pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
    pub.publish (*msg);
    ros::spinOnce ();
    loop_rate.sleep ();
  }
}
```



### Subscribing to point clouds

You may likewise subscribe to PCL point clouds using the standard
`ros::Subscriber`:



```cplusplus
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

void callback(const PointCloud::ConstPtr& msg)
{
  printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
  BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
    printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sub_pcl");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<PointCloud>("points2", 1, callback);
  ros::spin();
}
```



## ROS nodes

Several tools run as ROS nodes. They convert ROS messages or bags to and
from Point Cloud Data (PCD) file format.

<a id="bag_to_pcd"></a>

### bag_to_pcd

Reads a bag file, saving all ROS point cloud messages on a specified
topic as PCD files.

#### Usage



```
$ rosrun pcl_ros bag_to_pcd <input_file.bag> <topic> <output_directory>
```



Where:

- <input_file.bag> is the bag file name to read.
- <topic> is the topic in the bag file containing messages to save.
- <output_directory> is the directory on disk in which to create PCD
  files from the point cloud messages.

#### Example

Read messages from the `/laser_tilt_cloud` topic in `data.bag`,
saving a PCD file for each message into the `./pointclouds`
subdirectory.



```
$ rosrun pcl_ros bag_to_pcd data.bag /laser_tilt_cloud ./pointclouds
```



<a id="convert_pcd_to_image"></a>

### convert_pcd_to_image

Loads a PCD file, publishing it as a ROS image message five times a
second.

#### Published Topics

- `output`
  ([sensor_msgs/msg/Image](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/Image.html)):
  A stream of images generated from the PCD file.

#### Usage



```
$ rosrun pcl_ros convert_pcd_to_image <cloud.pcd>
```



Read the point cloud in <cloud.pcd> and publish it in ROS image
messages at 5Hz.

<a id="convert_pointcloud_to_image"></a>

### convert_pointcloud_to_image

Subscribes to a ROS point cloud topic and republishes image messages.

#### Subscribed Topics

- `input`
  ([sensor_msgs/msg/PointCloud2](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/PointCloud2.html)):
  A stream of point clouds to convert to images.

#### Published Topics

- `output`
  ([sensor_msgs/msg/Image](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/Image.html)):
  Corresponding stream of images.

#### Examples

Subscribe to the `/my_cloud` topic and republish each message on the
`/my_image` topic.



```
$ rosrun pcl_ros convert_pointcloud_to_image input:=/my_cloud output:=/my_image
```



To view the images created by the previous command, use
[image_view](https://index.ros.org/p/image_view).


```
$ rosrun image_view image_view image:=/my_image
```



<a id="pcd_to_pointcloud"></a>

### pcd_to_pointcloud

Loads a PCD file, publishing it one or more times as a ROS point cloud
message.

#### Published Topics

- `cloud_pcd`
  ([sensor_msgs/msg/PointCloud2](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/PointCloud2.html)):
  A stream of point clouds generated from the PCD file.

#### Parameters

- `~frame_id` (`str`, default: `/base_link`): Transform frame ID for
  published data.

#### Usage



```
$ rosrun pcl_ros pcd_to_pointcloud <file.pcd> [ <interval> ]
```



Where:

- <file.pcd> is the (required) file name to read.
- <interval> is the (optional) number of seconds to sleep between
  messages. If <interval> is zero or not specified the message is
  published once.

#### Examples

Publish the contents of `point_cloud_file.pcd` once in the
`/base_link` frame of reference.



```
$ rosrun pcl_ros pcd_to_pointcloud point_cloud_file.pcd
```



Publish the contents of `cloud_file.pcd` approximately ten times a
second in the `/odom` frame of reference.



```
$ rosrun pcl_ros pcd_to_pointcloud cloud_file.pcd 0.1 _frame_id:=/odom
```



<a id="pointcloud_to_pcd"></a>

### pointcloud_to_pcd

Subscribes to a ROS topic and saves point cloud messages to PCD files.
Each message is saved to a separate file, names are composed of an
optional `prefix` parameter, the ROS time of the message, and the
`.pcd` extension.

#### Subscribed Topics

- `input`
  ([sensor_msgs/msg/PointCloud2](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/PointCloud2.html)):
  A stream of point clouds to save as PCD files.

#### Parameters

- `~prefix` (`str`): Prefix for PCD file names created.
- `~fixed_frame` (`str`): If set, the transform from the fixed frame to
  the frame of the point cloud is written to the VIEWPOINT entry of the
  pcd file.
- `~binary` (`bool`, default: `false`): Output the pcd file in binary
  form.
- `~compressed` (`bool`, default: `false`): In case that binary output
  format is set, use binary compressed output.

#### Examples

Subscribe to the `/velodyne/pointcloud2` topic and save each message
in the current directory. File names look like
`1285627014.833100319.pcd`, the exact names depending on the message
time stamps.



```
$ rosrun pcl_ros pointcloud_to_pcd input:=/velodyne/pointcloud2
```



Set the `prefix` parameter in the current namespace, save messages to
files with names like `/tmp/pcd/vel_1285627015.132700443.pcd`.



```
$ rosrun pcl_ros pointcloud_to_pcd input:=/my_cloud _prefix:=/tmp/pcd/vel_
```



## More examples

We have more examples on <http://wiki.ros.org/pcl_ros/Tutorials> page

## Tutorials

- [Turning a PointCloud into an Image](#turning-a-pointcloud-into-an-image):
  This tutorial is a simple guide to turning a dense point cloud into an
  image message
- [ConvexHull planar surface reconstruction](#convexhull-planar-surface-reconstruction):
  ConvexHull planar surface reconstruction
- [ExtractPolygonalPrismData segmentation](#extractpolygonalprismdata-segmentation):
  ExtractPolygonalPrismData segmentation
- [PassThrough filtering](#passthrough-filtering):
  PassThrough filtering
- [SACSegmentationFromNormals planar segmentation](#sacsegmentationfromnormals-planar-segmentation):
  SACSegmentationFromNormals planar segmentation
- [VoxelGrid filtering](#voxelgrid-filtering):
  VoxelGrid filtering
- [Filter Nodelets](#filter-nodelets):
  Descriptions of many filter nodelets

Download the RViz display file here:
[table_scene_mug_textured.vcg](doc/table_scene_mug_textured.vcg) and
run: 

```
$ rosrun rviz rviz -d table_scene_mug_textured.vcg
```



![attachment:rviz.jpg](doc/rviz.jpg)

## Turning a PointCloud into an Image

- **Description:** This tutorial is a simple guide to turning a dense
  point cloud into an image message
- **Keywords:** kinect, pcl, opencv, openni
- **Tutorial Level:** BEGINNER

### Use ROS

Let's assume you have an openni camera up and running. Fire up a
terminal. 

```
$rostopic list
/camera/depth/points2
/camera/rgb/camera_info
/camera/rgb/image_color
/camera/rgb/image_mono
```



To convert the point cloud to an image, just run the following


```
rosrun pcl_ros convert_pointcloud_to_image input:=/camera/depth/points2 output:=/camera/depth/cloud_image
```



Then subscribe to the image and display it 

```
rosrun image_view image_view image:=/camera/depth/cloud_image
```



### Concept

Given a dense cloud, you can efficiently grab the rgb image from it as a
`sensor_msgs::Image`. The image message can then either be republished
or turned into an opencv `cv::Mat` using
[standard methods](http://www.ros.org/wiki/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages)


```cplusplus
//pcl::toROSMsg
#include <pcl/io/pcd_io.h>
//...
    try
    {
      pcl::toROSMsg (*cloud, image_); //convert the cloud
    }
    catch (std::runtime_error)
    {
    }
```



You may pass either `pcl::PointCloud<PointT>`, or a
`sensor_msgs::PointCloud2`.

### Code

See the `pcl_ros/src/tools/convert_pointcloud_to_image` and
`pcl_ros/src/tools/convert_pcd_to_image` for example code.



```cplusplus
// ROS core
#include <ros/ros.h>
//Image message
#include <sensor_msgs/Image.h>
//pcl::toROSMsg
#include <pcl/io/pcd_io.h>
//stl stuff
#include <string>

class PointCloudToImage
{
public:
  void
  cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud)
  {
    if ((cloud->width * cloud->height) == 0)
      return; //return if the cloud is not dense!
    try
    {
      pcl::toROSMsg (*cloud, image_); //convert the cloud
    }
    catch (std::runtime_error e)
    {
      ROS_ERROR_STREAM("Error in converting cloud to image message: "
                        << e.what());
    }
    image_pub_.publish (image_); //publish our cloud image
  }
  PointCloudToImage () : cloud_topic_("input"),image_topic_("output")
  {
    sub_ = nh_.subscribe (cloud_topic_, 30,
                          &PointCloudToImage::cloud_cb, this);
    image_pub_ = nh_.advertise<sensor_msgs::Image> (image_topic_, 30);

    //print some info about the node
    std::string r_ct = nh_.resolveName (cloud_topic_);
    std::string r_it = nh_.resolveName (image_topic_);
    ROS_INFO_STREAM("Listening for incoming data on topic " << r_ct );
    ROS_INFO_STREAM("Publishing image on topic " << r_it );
  }
private:
  ros::NodeHandle nh_;
  sensor_msgs::Image image_; //cache the image message
  std::string cloud_topic_; //default input
  std::string image_topic_; //default output
  ros::Subscriber sub_; //cloud subscriber
  ros::Publisher image_pub_; //image message publisher
};

int
main (int argc, char **argv)
{
  ros::init (argc, argv, "convert_pointcloud_to_image");
  PointCloudToImage pci; //this loads up the node
  ros::spin (); //where she stops nobody knows
  return 0;
}
```

## ConvexHull planar surface reconstruction

- **Description:** ConvexHull planar surface reconstruction
- **Tutorial Level:** BEGINNER

The following launch file starts a nodelet manager together with a
`VoxelGrid` PCL filter nodelet. The input `PointCloud2` topic is set
to */camera/depth/points*.

The default filtering values are set to filter data on the **z**-axis
between **0.01** and **1.5** meters, and downsample the data with a leaf
size of **0.01** meters.

The segmentation is performed by first estimating surface normals at
each point for a support of **0.015** meters, and then using a
RANSAC-based estimator for fitting planes with a threshold of **0.1**
meters, and a normal deviation of \*\*5 **degrees (**0.09 **radians).**

The convex hull of the projected planar inliers is estimated and
published on the */convex_hull/output* topic.



```python
<launch>
  <node pkg="nodelet" type="nodelet" name="pcl_manager" args="manager" output="screen" />

  <!-- Run a VoxelGrid filter to clean NaNs and downsample the data -->
  <node pkg="nodelet" type="nodelet" name="voxel_grid" args="load pcl/VoxelGrid pcl_manager" output="screen">
    <remap from="~input" to="/camera/depth/points" />
    <rosparam>
      filter_field_name: z
      filter_limit_min: 0.01
      filter_limit_max: 1.5
      filter_limit_negative: False
      leaf_size: 0.01
    </rosparam>
  </node>

  <!-- Estimate point normals -->
  <node pkg="nodelet" type="nodelet" name="normal_estimation" args="load pcl/NormalEstimation pcl_manager" output="screen">
    <remap from="~input" to="/voxel_grid/output" />
    <rosparam>
      # -[ Mandatory parameters
      k_search: 0
      radius_search: 0.015
      # Set the spatial locator. Possible values are: 0 (ANN), 1 (FLANN), 2 (organized)
      spatial_locator: 0
    </rosparam>
  </node>

  <!-- Segment the table plane -->
  <node pkg="nodelet" type="nodelet" name="planar_segmentation" args="load pcl/SACSegmentationFromNormals pcl_manager" output="screen">
    <remap from="~input"   to="/voxel_grid/output" />
    <remap from="~normals" to="/normal_estimation/output" />
    <rosparam>
      # -[ Mandatory parameters

      # model_type:
      # 0: SACMODEL_PLANE
      # 1: SACMODEL_LINE
      # 2: SACMODEL_CIRCLE2D
      # 3: SACMODEL_CIRCLE3D
      # 4: SACMODEL_SPHERE
      # 5: SACMODEL_CYLINDER
      # 6: SACMODEL_CONE
      # 7: SACMODEL_TORUS
      # 8: SACMODEL_PARALLEL_LINE
      # 9: SACMODEL_PERPENDICULAR_PLANE
      # 10: SACMODEL_PARALLEL_LINES
      # 11: SACMODEL_NORMAL_PLANE
      # 12: SACMODEL_NORMAL_SPHERE
      # 13: SACMODEL_REGISTRATION
      # 14: SACMODEL_REGISTRATION_2D
      # 15: SACMODEL_PARALLEL_PLANE
      # 16: SACMODEL_NORMAL_PARALLEL_PLANE
      # 17: SACMODEL_STICK
      model_type: 11
      distance_threshold: 0.01
      max_iterations: 1000
      method_type: 0
      optimize_coefficients: true
      normal_distance_weight: 0.1
      eps_angle: 0.09
    </rosparam>
  </node>

  <node pkg="nodelet" type="nodelet" name="extract_plane_indices" args="load pcl/ExtractIndices pcl_manager" output="screen">
    <remap from="~input"   to="/voxel_grid/output" />
    <remap from="~indices" to="/planar_segmentation/inliers" />
    <rosparam>
      negative: true
    </rosparam>
  </node>

  <!-- Project the planar inliers -->
  <node pkg="nodelet" type="nodelet" name="project_plane_inliers" args="load pcl/ProjectInliers pcl_manager" output="screen">
   <remap from="~input"   to="/voxel_grid/output" />
   <remap from="~indices" to="/planar_segmentation/inliers" />
   <remap from="~model"   to="/planar_segmentation/model" />
   <rosparam>
     model_type: 11
     copy_all_data: false
     copy_all_fields: true
   </rosparam>
  </node>

  <!-- Compute the convex hull -->
  <node pkg="nodelet" type="nodelet" name="convex_hull" args="load pcl/ConvexHull2D pcl_manager" output="screen">
    <remap from="~input"   to="/project_plane_inliers/output" />
  </node>
</launch>
```



![ pic1 display view](http://wiki.ros.org/pcl_ros/Tutorials/ConvexHull%20planar%20surface%20reconstruction?action=AttachFile&do=get&target=pic1.png )

## ExtractPolygonalPrismData segmentation

- **Note:** This tutorial assumes that you have completed the previous
  tutorials: ExtractPolygonalPrismData segmentation.
- **Description:** ExtractPolygonalPrismData segmentation

The following launch file starts a nodelet manager together with a
`VoxelGrid` PCL filter nodelet. The input `PointCloud2` topic is set
to */scene_pointcloud2*.

The default filtering values are set to filter data on the **z**-axis
between **0.01** and **1.5** meters, and downsample the data with a leaf
size of **0.01** meters.

The segmentation is performed by first estimating surface normals at
each point for a support of **0.015** meters, and then using a
RANSAC-based estimator for fitting planes with a threshold of **0.1**
meters, and a normal deviation of \*\*5 **degrees (**0.09 **radians).**

The convex hull of the projected planar inliers is estimated and an
`ExtractPolygonalPrismData` nodelet is used to extract all point
indices between **0** and **0.5** meters from the planar table support,
with their projections falling inside the convex hull.



```python
<launch>
  <node pkg="nodelet" type="nodelet" name="pcl_manager" args="manager" output="screen" />

  <!-- Run a VoxelGrid filter to clean NaNs and downsample the data -->
  <node pkg="nodelet" type="nodelet" name="voxel_grid" args="load pcl/VoxelGrid pcl_manager" output="screen">
    <remap from="~input" to="/scene_pointcloud2" />
    <rosparam>
      filter_field_name: z
      filter_limit_min: 0.01
      filter_limit_max: 1.5
      filter_limit_negative: False
      leaf_size: 0.01
    </rosparam>
  </node>

  <!-- Estimate point normals -->
  <node pkg="nodelet" type="nodelet" name="normal_estimation" args="load pcl/NormalEstimation pcl_manager" output="screen">
    <remap from="~input" to="/voxel_grid/output" />
    <rosparam>
      # -[ Mandatory parameters
      k_search: 0
      radius_search: 0.015
      # Set the spatial locator. Possible values are: 0 (ANN), 1 (FLANN), 2 (organized)
      spatial_locator: 0
    </rosparam>
  </node>

  <!-- Segment the table plane -->
  <node pkg="nodelet" type="nodelet" name="planar_segmentation" args="load pcl/SACSegmentationFromNormals pcl_manager" output="screen">
    <remap from="~input"   to="/voxel_grid/output" />
    <remap from="~normals" to="/normal_estimation/output" />
    <rosparam>
      # -[ Mandatory parameters
      model_type: 11
      distance_threshold: 0.1
      max_iterations: 1000
      method_type: 0
      optimize_coefficients: true
      normal_distance_weight: 0.1
      eps_angle: 0.09
    </rosparam>
  </node>

  <node pkg="nodelet" type="nodelet" name="extract_plane_indices" args="load pcl/ExtractIndices pcl_manager" output="screen">
    <remap from="~input"   to="/voxel_grid/output" />
    <remap from="~indices" to="/planar_segmentation/inliers" />
    <rosparam>
      negative: true
    </rosparam>
  </node>

  <!-- Project the planar inliers -->
  <node pkg="nodelet" type="nodelet" name="project_plane_inliers" args="load pcl/ProjectInliers pcl_manager" output="screen">
   <remap from="~input"   to="/voxel_grid/output" />
   <remap from="~indices" to="/planar_segmentation/inliers" />
   <remap from="~model"   to="/planar_segmentation/model" />
   <rosparam>
     model_type: 11
     copy_all_data: false
     copy_all_fields: true
   </rosparam>
  </node>

  <!-- Compute the convex hull -->
  <node pkg="nodelet" type="nodelet" name="convex_hull" args="load pcl/ConvexHull2D pcl_manager" output="screen">
    <remap from="~input"   to="/project_plane_inliers/output" />
  </node>

  <!-- Extract the object clusters using a polygonal prism -->
  <node pkg="nodelet" type="nodelet" name="extract_objects_table" args="load pcl/ExtractPolygonalPrismData pcl_manager" output="screen">
   <remap from="~input"         to="/extract_plane_indices/output" />
   <remap from="~planar_hull"   to="/convex_hull/output" />
   <rosparam>
     height_min: 0
     height_max: 0.5
   </rosparam>
  </node>

  <node pkg="nodelet" type="nodelet" name="extract_objects_indices" args="load pcl/ExtractIndices pcl_manager" output="screen">
    <!-- Extract_plane_indices needs to be negated for this work -->
    <remap from="~input"   to="/extract_plane_indices/output" />
    <remap from="~indices" to="/extract_objects_table/output" />
    <rosparam>
      negative: false
    </rosparam>
  </node>
</launch>
```

## PassThrough filtering

- **Description:** PassThrough filtering
- **Tutorial Level:** BEGINNER

The following launch file starts a nodelet manager together with a
`PassThrough` PCL filter nodelet. The input `PointCloud2` topic is
set to */scene_pointcloud2*.

The default filtering values are set to filter data on the **z**-axis
between **0.01** and **1.5** meters.



```
<launch>
  <node pkg="nodelet" type="nodelet" name="pcl_manager" args="manager" output="screen" />

  <!-- Run a passthrough filter to clean NaNs -->
  <node pkg="nodelet" type="nodelet" name="passthrough" args="load pcl/PassThrough pcl_manager" output="screen">
    <remap from="~input" to="/camera/depth/points" />
    <rosparam>
      filter_field_name: z
      filter_limit_min: 0.01
      filter_limit_max: 1.5
      filter_limit_negative: False
    </rosparam>
  </node>
</launch>
```

 In the first terminal,



```
roscore
```

 Open a new shell, and put following command.



```
roslaunch openni2_launch openni2.launch
roslaunch pcl_ros_tutorials pass_through_filtering.launch
```

 Then, launch rviz.



```
rviz
```

 Then, you push "Add" button and choose
"PointCloud2".

![rocon_remocon display view](http://wiki.ros.org/pcl_ros/Tutorials/PassThrough filtering?action=AttachFile&do=get&target=Add.png)

You can change topic. ![attachment:pic1.png](doc/pic1.png)

The following image is passthrough filter.
![attachment:pic2.png](doc/pic2.png)

## SACSegmentationFromNormals planar segmentation

- **Description:** SACSegmentationFromNormals planar segmentation
- **Tutorial Level:** BEGINNER

The following launch file starts a nodelet manager together with a
`VoxelGrid` PCL filter nodelet. The input `PointCloud2` topic is set
to */camera/depth/points*.

The default filtering values are set to filter data on the **z**-axis
between **0.01** and **1.5** meters, and downsample the data with a leaf
size of **0.01** meters.

The segmentation is performed by first estimating surface normals at
each point for a support of **0.015** meters, and then using a
RANSAC-based estimator for fitting planes with a threshold of **0.1**
meters, and a normal deviation of \*\*5 **degrees (**0.09 **radians).**



```python
<launch>
  <node pkg="nodelet" type="nodelet" name="pcl_manager" args="manager" output="screen" />

  <!-- Run a VoxelGrid filter to clean NaNs and downsample the data -->
  <node pkg="nodelet" type="nodelet" name="voxel_grid" args="load pcl/VoxelGrid pcl_manager" output="screen">
    <remap from="~input" to="/camera/depth/points" />
    <rosparam>
      filter_field_name: z
      filter_limit_min: 0.01
      filter_limit_max: 1.5
      filter_limit_negative: False
      leaf_size: 0.01
    </rosparam>
  </node>

  <!-- Estimate point normals -->
  <node pkg="nodelet" type="nodelet" name="normal_estimation" args="load pcl/NormalEstimation pcl_manager" output="screen">
    <remap from="~input" to="/voxel_grid/output" />
    <rosparam>
      # -[ Mandatory parameters
      k_search: 0
      radius_search: 0.015
      # Set the spatial locator. Possible values are: 0 (ANN), 1 (FLANN), 2 (organized)
      spatial_locator: 0
    </rosparam>
  </node>

  <!-- Segment the table plane -->
  <node pkg="nodelet" type="nodelet" name="planar_segmentation" args="load pcl/SACSegmentationFromNormals pcl_manager" output="screen">
    <remap from="~input"   to="/voxel_grid/output" />
    <remap from="~normals" to="/normal_estimation/output" />
    <rosparam>
      # -[ Mandatory parameters
      # model_type:
      # 0: SACMODEL_PLANE
      # 1: SACMODEL_LINE
      # 2: SACMODEL_CIRCLE2D
      # 3: SACMODEL_CIRCLE3D
      # 4: SACMODEL_SPHERE
      # 5: SACMODEL_CYLINDER
      # 6: SACMODEL_CONE
      # 7: SACMODEL_TORUS
      # 8: SACMODEL_PARALLEL_LINE
      # 9: SACMODEL_PERPENDICULAR_PLANE
      # 10: SACMODEL_PARALLEL_LINES
      # 11: SACMODEL_NORMAL_PLANE
      # 12: SACMODEL_NORMAL_SPHERE
      # 13: SACMODEL_REGISTRATION
      # 14: SACMODEL_REGISTRATION_2D
      # 15: SACMODEL_PARALLEL_PLANE
      # 16: SACMODEL_NORMAL_PARALLEL_PLANE
      # 17: SACMODEL_STICK
      model_type: 11
      distance_threshold: 0.1
      max_iterations: 1000
      method_type: 0
      optimize_coefficients: true
      normal_distance_weight: 0.1
      eps_angle: 0.09
    </rosparam>
  </node>

  <node pkg="nodelet" type="nodelet" name="extract_plane_indices" args="load pcl/ExtractIndices pcl_manager" output="screen">
    <remap from="~input"   to="/voxel_grid/output" />
    <remap from="~indices" to="/planar_segmentation/inliers" />
    <rosparam>
      negative: true
    </rosparam>
  </node>
</launch>
```



In the first terminal,



```
roscore
```

 Open a new shell, and put following command.



```
roslaunch openni2_launch openni2.launch
roslaunch pcl_ros_tutorials SAC.launch
```

 Then, launch rviz.



```
rviz
```

 Then, you push "Add" button and choose
"PointCloud2".

![attachment:pic1.png](doc/pcl_ros_Tutorials_SACSegmentationFromNormals%20planar%20segmentation_pic1.png)

The following image is image extracted plane.
![attachment:pic2.png](doc/pcl_ros_Tutorials_SACSegmentationFromNormals%20planar%20segmentation_pic2.png)

## VoxelGrid filtering

- **Description:** VoxelGrid filtering
- **Tutorial Level:** BEGINNER

The following launch file starts a nodelet manager together with a
`VoxelGrid` PCL filter nodelet. The input `PointCloud2` topic is set
to */camera/depth/points*.

The default filtering values are set to filter data on the **z**-axis
between **0.01** and **1.5** meters, and downsample the data with a leaf
size of **0.01** meters.



```python
<launch>
  <node pkg="nodelet" type="nodelet" name="pcl_manager" args="manager" output="screen" />

  <!-- Run a VoxelGrid filter to clean NaNs and downsample the data -->
  <node pkg="nodelet" type="nodelet" name="voxel_grid" args="load pcl/VoxelGrid pcl_manager" output="screen">
    <remap from="~input" to="/camera/depth/points" />
    <rosparam>
      filter_field_name: z
      filter_limit_min: 0.01
      filter_limit_max: 1.5
      filter_limit_negative: False
      leaf_size: 0.01
    </rosparam>
  </node>
</launch>
```



In the first terminal,



```
roscore
```

 Open a new shell, and put following command.



```
roslaunch openni2_launch openni2.launch
roslaunch pcl_ros_tutorials voxed_grid_filtering.launch
```

 Then, launch rviz.



```
rviz
```



![attachment:pic1.png](doc/pcl_ros_Tutorials_VoxelGrid%20filtering_pic1.png)

VoxelGrid is enable on the following image.

![attachment:pic2.png](doc/pcl_ros_Tutorials_VoxelGrid%20filtering_pic2.png)

## Filter Nodelets

### Users

#### Extract Indices

The `ExtractIndices` nodelet wraps the
[pcl::ExtractIndices](https://pointclouds.org/documentation/classpcl_1_1_extract_indices.html)
filter (also see
[tutorial](https://pcl.readthedocs.io/projects/tutorials/en/latest/extract_indices.html#extract-indices)).

The following
[dynamic_reconfigure](https://index.ros.org/p/dynamic_reconfigure)
parameters are used:

##### Parameters

###### Dynamically Reconfigurable Parameters

See the
[dynamic_reconfigure](https://index.ros.org/p/dynamic_reconfigure)
package for details on dynamically reconfigurable parameters.

- `~negative` (`bool`, default: `False`): Extract indices or the
  negative (all-indices)

#### PassThrough

The `PassThrough` nodelet wraps the
[pcl::PassThrough](https://pointclouds.org/documentation/classpcl_1_1_pass_through.html)
filter (also see
[PassThrough tutorial](https://pcl.readthedocs.io/projects/tutorials/en/latest/passthrough.html)).

The following
[dynamic_reconfigure](https://index.ros.org/p/dynamic_reconfigure)
parameters are used:

##### Parameters

###### Dynamically Reconfigurable Parameters

See the
[dynamic_reconfigure](https://index.ros.org/p/dynamic_reconfigure)
package for details on dynamically reconfigurable parameters.

- `~filter_field_name` (`str`, default: `z`): The field name used for
  filtering
- `~filter_limit_min` (`double`, default: `0.0`): The minimum allowed
  field value a point will be considered from Range: -1000.0 to 1000.0
- `~filter_limit_max` (`double`, default: `1.0`): The maximum allowed
  field value a point will be considered from Range: -1000.0 to 1000.0
- `~filter_limit_negative` (`bool`, default: `False`): Set to true if we
  want to return the data outside \[filter_limit_min;
  filter_limit_max\].
- `~input_frame` (`str`): The input TF frame the data should be
  transformed into before processing, if input.header.frame_id is
  different.
- `~output_frame` (`str`): The output TF frame the data should be
  transformed into after processing, if input.header.frame_id is
  different.

#### ProjectInliers

The `ProjectInliers` nodelet wraps the
[pcl::ProjectInliers](https://pointclouds.org/documentation/classpcl_1_1_project_inliers.html)
filter (also see
[Projection tutorial](https://pcl.readthedocs.io/projects/tutorials/en/latest/project_inliers.html)).

#### RadiusOutlierRemoval

The `RadiusOutlierRemoval` nodelet wraps the
[pcl::RadiusOutlierRemoval](https://pointclouds.org/documentation/classpcl_1_1_radius_outlier_removal.html)
filter (also see
[Outlier removal tutorial](https://pcl.readthedocs.io/projects/tutorials/en/latest/remove_outliers.html)).

#### StatisticalOutlierRemoval

The `StatisticalOutlierRemoval` nodelet wraps the
[pcl::StatisticalOutlierRemoval](https://pointclouds.org/documentation/classpcl_1_1_statistical_outlier_removal.html)
filter (also see
[Statistical outlier removal tutorial](https://pcl.readthedocs.io/projects/tutorials/en/latest/statistical_outlier.html)).

The following
[dynamic_reconfigure](https://index.ros.org/p/dynamic_reconfigure)
parameters are used:

##### Parameters

###### Dynamically Reconfigurable Parameters

See the
[dynamic_reconfigure](https://index.ros.org/p/dynamic_reconfigure)
package for details on dynamically reconfigurable parameters.

- `~mean_k` (`int`, default: `2`): The number of points (k) to use for
  mean distance estimation Range: 2 to 100
- `~stddev` (`double`, default: `0.0`): The standard deviation
  multiplier threshold. All points outside the mean +- sigma \* std_mul
  will be considered outliers. Range: 0.0 to 5.0
- `~negative` (`bool`, default: `False`): Set whether the inliers should
  be returned (true) or the outliers (false)

#### VoxelGrid

The `VoxelGrid` nodelet wraps the
[pcl::VoxelGrid](https://pointclouds.org/documentation/classpcl_1_1_voxel_grid.html)
filter (also see
[Downsampling tutorial](https://pcl.readthedocs.io/projects/tutorials/en/latest/voxel_grid.html)).

The following
[dynamic_reconfigure](https://index.ros.org/p/dynamic_reconfigure)
parameters are used:

##### Parameters

###### Dynamically Reconfigurable Parameters

See the
[dynamic_reconfigure](https://index.ros.org/p/dynamic_reconfigure)
package for details on dynamically reconfigurable parameters.

- `~leaf_size` (`double`, default: `0.01`): The size of a leaf (on
  x,y,z) used for downsampling. Range: 0.0 to 1.0
- `~filter_field_name` (`str`, default: `z`): The field name used for
  filtering
- `~filter_limit_min` (`double`, default: `0.0`): The minimum allowed
  field value a point will be considered from Range: -1000.0 to 1000.0
- `~filter_limit_max` (`double`, default: `1.0`): The maximum allowed
  field value a point will be considered from Range: -1000.0 to 1000.0
- `~filter_limit_negative` (`bool`, default: `False`): Set to true if we
  want to return the data outside \[filter_limit_min;
  filter_limit_max\].
- `~input_frame` (`str`): The input TF frame the data should be
  transformed into before processing, if input.header.frame_id is
  different.
- `~output_frame` (`str`): The output TF frame the data should be
  transformed into after processing, if input.header.frame_id is
  different.

#### CropBox

The `CropBox` nodelet wraps the
[pcl::CropBox](https://pointclouds.org/documentation/classpcl_1_1_crop_box.html)
filter.

The following
[dynamic_reconfigure](https://index.ros.org/p/dynamic_reconfigure)
parameters are used:

##### Parameters

###### Dynamically Reconfigurable Parameters

- `~min_x` (`double`, default: `-1.0`): The minimum allowed x value a
  point will be considered from. Range: -1000.0 to 1000.0
- `~max_x` (`double`, default: `1.0`): The maximum allowed x value a
  point will be considered from. Range: -1000.0 to 1000.0
- `~min_y` (`double`, default: `-1.0`): The minimum allowed y value a
  point will be considered from. Range: -1000.0 to 1000.0
- `~max_y` (`double`, default: `1.0`): The maximum allowed y value a
  point will be considered from. Range: -1000.0 to 1000.0
- `~min_z` (`double`, default: `-1.0`): The minimum allowed z value a
  point will be considered from. Range: -1000.0 to 1000.0
- `~max_z` (`double`, default: `1.0`): The maximum allowed z value a
  point will be considered from. Range: -1000.0 to 1000.0
- `~keep_organized` (`bool`, default: `False`): If true, the pointcloud
  shape (width, height) will be retained and filtered-out points will be
  set to nan. If false, pointcloud is reshaped to height 1 and
  filtered-out points are removed. Please not this is broken in Melodic
  and the pointcloud is always unorganized. In Noetic, it works
  correctly.
- `~negative` (`bool`, default: `False`): Set to true if we want to
  return the data outside the limiting box. If false, data inside the
  box are returned.
- `~input_frame` (`str`): The input TF frame the data should be
  transformed into before processing, if input.header.frame_id is
  different.
- `~output_frame` (`str`): The output TF frame the data should be
  transformed into after processing, if input.header.frame_id is
  different.

### Developers

All pcl_ros nodelet filters inherit from
`pcl_ros::Filter`, which requires that any class inheriting from it
implement the following interface:

- `child_init()`, `filter()`, `onInit()` and `config_callback()`
  are all virtual and can be overridden

<!-- -->

- `filter (PointCloud2 &output)` is pure abstract and must be
  implemented

<!-- -->

- if `child_init (ros::NodeHandle &nh, bool &has_service)` is
  implemented, and `has_service` is set to **true**, the `Filter`
  base class `dynamic_reconfigure` service will not be created

#### Common ROS parameters

The following ROS parameters are usually read on startup for any
pcl_ros nodelet.

- **max_queue_size**, *int* : defines the maximum queue size the ROS
  subscribers and publishers in the nodelet should use (see
  [ros::NodeHandle::subscribe](http://www.ros.org/doc/api/roscpp/html/classros_1_1NodeHandle.html#a0b8398fd50b8f7b3cc44f642252e4fea)
  for more information).

<!-- -->

- **use_indices**, *bool* : usually indicates whether the nodelet should
  listen for incoming data on **indices**. This data represents a subset
  of point indices (`pcl::PointIndices` message) that should be used
  from the input dataset.

<!-- -->

- **approximate_sync**, *bool* : indicates whether the input topics that
  the nodelet listens to, should synchronize with an
  `ApproximateTimeSynchronizer` (their `header.stamp` can differ) or
  an `ExactTimeSynchronizer` (their `header.stamp` must be
  identical)

#### Input/Output topics

The following list of ROS topics are used by all
pcl_ros filters:

- **input**, in, [PointCloud2](https://index.ros.org/p/sensor_msgs) :
  input point cloud data to filter.
- **indices**, in, [pcl/PointIndices](pcl/PointIndices) : input point
  cloud data indices to use from **input**.

<!-- -->

- **output**, out, [PointCloud2](https://index.ros.org/p/sensor_msgs) :
  the resultant filtered point cloud data.
