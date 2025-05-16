^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pcl_conversions
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.7.1 (2025-05-16)
------------------
* Bump minimum cmake version to 3.20 (`#496 <https://github.com/ros-perception/perception_pcl/issues/496>`_)
* Contributors: Ramon Wijnands

2.7.0 (2025-05-15)
------------------
* Modern CMake (`#489 <https://github.com/ros-perception/perception_pcl/issues/489>`_)
* Contributors: Patrick Roncagliolo

2.6.3 (2025-04-07)
------------------
* Update for ROS 2 file name changes (`#472 <https://github.com/ros-perception/perception_pcl/issues/472>`_)
* Contributors: Steve Macenski

2.6.2 (2024-11-07)
------------------
* sorts pc msg fields by offset (`#438 <https://github.com/ros-perception/perception_pcl/issues/438>`_)
* Fix `Could NOT find Boost (missing: Boost_INCLUDE_DIR)` `#452 <https://github.com/ros-perception/perception_pcl/issues/452>`_ (`#454 <https://github.com/ros-perception/perception_pcl/issues/454>`_)
* In PCL 1.14.1 and newer, generate smaller point cloud msgs (`#450 <https://github.com/ros-perception/perception_pcl/issues/450>`_)
* Switch to build_export_depend for libpcl-all-dev (`#447 <https://github.com/ros-perception/perception_pcl/issues/447>`_)
* Contributors: Marco Salathe, Markus Vieth, Ramon Wijnands, Yadu

1.6.2 (2018-05-20)
------------------

1.6.1 (2018-05-08)
------------------
* Add 1.6.0 section to CHANGELOG.rst
* Use foreach + string regex to implement list(filter on old cmake
* Downgrade the required cmake version for backward compatibility
* update package.xml links to point to new repository
* CMake 3.6.3 is sufficient
* Fix a bug building on artful.
* Fixup pcl_conversions test
* Contributors: Chris Lalancette, Kentaro Wada, Mikael Arguedas, Paul Bovbel

1.6.0 (2018-04-30)
------------------

* Fix build and update maintainers
* Contributors: Paul Bovbel, Kentaro Wada

0.2.1 (2015-06-08)
------------------
* Added a test for rounding errors in stamp conversion
  for some values the test fails.
* add pcl::PointCloud to Image msg converter for extracting the rgb component of a cloud
* Contributors: Brice Rebsamen, Lucid One, Michael Ferguson, Paul Bovbel

0.2.0 (2014-04-10)
------------------
* Added conversions for stamp types
* update maintainer info, add eigen dependency
* fix Eigen dependency
* Make pcl_conversions run_depend on libpcl-all-dev
* Contributors: Brice Rebsamen, Paul Bovbel, Scott K Logan, William Woodall

0.1.5 (2013-08-27)
------------------
* Use new pcl rosdep keys (libpcl-all and libpcl-all-dev)

0.1.4 (2013-07-13)
------------------
* Fixup dependencies and CMakeLists.txt:

  * Added a versioned dependency on pcl, fixes `#1 <https://github.com/ros-perception/pcl_conversions/issues/1>`_
  * Added a dependency on pcl_msgs, fixes `#2 <https://github.com/ros-perception/pcl_conversions/issues/2>`_
  * Wrapped the test target in a CATKIN_ENABLE_TESTING check

0.1.3 (2013-07-13)
------------------
* Add missing dependency on roscpp
* Fixup tests and pcl usage in CMakeList.txt

0.1.2 (2013-07-12)
------------------
* small fix for conversion functions

0.1.1 (2013-07-10)
------------------
* Fix find_package bug with pcl

0.1.0 (2013-07-09 21:49:26 -0700)
---------------------------------
- Initial release
- This package is designed to allow users to more easily convert between pcl-1.7+ types and ROS message types
