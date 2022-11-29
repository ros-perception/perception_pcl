#
# Copyright (c) 2022, Carnegie Mellon University
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of Carnegie Mellon University nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

import time
import unittest
import inspect
import yaml
import sys
from threading import Thread, Event

import os
import ament_index_python

import rclpy
import rclpy.node

from sensor_msgs.msg import PointCloud2

from launch_ros.actions import Node
import launch
import launch.actions
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.util
import launch_testing.legacy

import pytest


@pytest.mark.launch_test
def generate_test_description():
    dummy_publisher = os.getenv('DUMMY_PUBLISHER')
    filter_name = os.getenv('FILTER_NAME')
    class_name = os.getenv('CLASS_NAME')
    extra_ros_args = []
    if os.getenv('EXTRA_ROS_ARGS'):
        extra_ros_args = os.getenv('EXTRA_ROS_ARGS').split(" ")

    dummy_node = launch.actions.ExecuteProcess(
        cmd=[dummy_publisher],
        name='dummy_point_cloud_node',
        output='screen'
    )

    filter_node = launch_ros.actions.Node(
        package='pcl_ros',
        executable="filter_{}_node".format(filter_name),
        output="screen",
        ros_arguments=[#"--log-level", "{}Node:=debug".format(class_name), 
                       "-r", "input:=point_cloud2"] + extra_ros_args
    )

    return launch.LaunchDescription([
        dummy_node,
        filter_node,
        launch_testing.actions.ReadyToTest()
    ]), {}


class TestFilters(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Initialize the ROS context for the test node
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        # Shutdown the ROS context
        rclpy.shutdown()

    def setUp(self):
        # Create a ROS node for tests
        self.node = rclpy.create_node('input_output_node')

    def tearDown(self):
        self.node.destroy_node()

    def spin(self):
        print("spin")
        rclpy.spin(self.node)
        print("spin end")

    def test_output(self):
        try:
            node = TestNode('test_node')
            node.start_subscriber()
            msgs_received_flag = node.msg_event_object.wait(timeout=5.0)
            assert msgs_received_flag, 'Did not receive msgs !'
        finally:
            node.stop()

    def test_output_resubscribe(self):
        try:
            node = TestNode('test_node')
            node.start_subscriber()
            msgs_received_flag = node.msg_event_object.wait(timeout=5.0)
            assert msgs_received_flag, 'Did not receive msgs !'
            node.stop_subscriber()
            time.sleep(1)
            node.start_subscriber()
            msgs_received_flag = node.msg_event_object.wait(timeout=5.0)
            assert msgs_received_flag, 'Did not receive msgs !'
        finally:
            node.stop()

class TestNode(rclpy.node.Node):
    def __init__(self, name='test_node'):
        super().__init__(name)

        # Add a spin thread
        self.alive = True
        self.ros_spin_thread = Thread(target=self.spin)
        self.ros_spin_thread.start()

    def spin(self):
        while self.alive:
            rclpy.spin_once(self)
            time.sleep(0.01)
    
    def stop(self):
        self.alive = False
        self.destroy_node()

    def start_subscriber(self):
        # Create a subscriber
        self.msg_event_object = Event()
        self.subscription = self.create_subscription(
            PointCloud2,
            'output',
            self.subscriber_callback,
            10
        )

    def stop_subscriber(self):
        self.destroy_subscription(self.subscription)
        self.subscription = None


    def subscriber_callback(self, data):
        self.msg_event_object.set()
