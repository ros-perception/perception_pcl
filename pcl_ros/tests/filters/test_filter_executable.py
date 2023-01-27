import os
import unittest

import launch
import launch.actions
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
import pytest

from launch_testing_ros import WaitForTopics
from sensor_msgs.msg import PointCloud2


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    dummy_plugin = os.getenv('DUMMY_PLUGIN')
    filter_executable = os.getenv('FILTER_EXECUTABLE')
    use_indices = os.getenv('USE_INDICES')
    approximate_sync = os.getenv('APPROXIMATE_SYNC')

    ros_arguments = ["-r", "input:=point_cloud2"]
    if use_indices:
        ros_arguments.extend(["-p", "use_indices:={}".format(use_indices)])
    if approximate_sync:
        ros_arguments.extend(["-p", "approximate_sync:={}".format(approximate_sync)])

    return launch.LaunchDescription([
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'component', 'standalone', 'pcl_ros_tests_filters', dummy_plugin],
            name='dummy_topics_node',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='pcl_ros',
            executable=filter_executable,
            output="screen",
            ros_arguments=ros_arguments
        ),
        launch_testing.actions.ReadyToTest()
    ])


class TestFilter(unittest.TestCase):
    def test_filter_output(self):
        wait_for_topics = WaitForTopics([('output', PointCloud2)], timeout=5.0)
        assert wait_for_topics.wait()
        assert 'output' in wait_for_topics.topics_received(), "Didn't receive message"
        wait_for_topics.shutdown()
