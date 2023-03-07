
import time
import unittest
import inspect
import yaml

import os
import ament_index_python

import rclpy

from ella_interfaces.msg import FullSteering
from geometry_msgs.msg import PoseStamped

from launch_ros.actions import Node
import launch
import launch_testing
import launch_testing.actions
import launch_testing.util

import subprocess

import pytest


@pytest.mark.launch_test
def generate_test_description():
    """
    Specify nodes or processes to launch for test.

    :param -
    :return dut [ros2 node] node to be tested (device under test)
    :return ...,... specifications for launch_testing

    Multiple nodes that are to be tested can be launched
    """
    # dut -> device under test is the node to be tested in this example
    dut = Node(
        package='shipmodel3dof',
        executable='shipmodel3dof',
        name='shipmodel3dofl',
    )
    context = {'dut': dut}

    return (launch.LaunchDescription([
        dut,
        launch_testing.actions.ReadyToTest()]
        ), context
    )

class TestProcessOutput(unittest.TestCase):
    """
    Details to use this class in the context of launch_testing.

    nodes: https://github.com/ros2/launch_ros
    process: https://github.com/ros2/launch/tree/master/launch_testing
    """

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

    def timer_callback(self):
        """
        Set velocity and rudder angle and publish the data to ros2.

        :param -
        :return -
        """
        # Read input data that is send to dut
        INPUT_DATA_PATH = os.path.join(
            ament_index_python.get_package_prefix('shipmodel3dof'),
            'lib/shipmodel3dof',
            'input_data.yaml')

        with open(INPUT_DATA_PATH) as f:
            data = yaml.safe_load(f)

        msg = FullSteering()
        msg.rudder_port = data['msg']['rudder_port']
        msg.rpm_port = data['msg']['rpm_port']
        msg.rpm_starboard = 0.0
        msg.rpm_bow = 0.0 
        msg.rudder_starboard = 0.0 
        msg.rudder_bow = 0.0

        self.publisher_.publish(msg)

    def test_1_nomoto(self):
        
        # Change directory to nomoto test folder in package build directory. 
        # Following processes will execute in this directory 
        install_path = ament_index_python.get_package_prefix('shipmodel3dof')
        build_path = "build/shipmodel3dof/test/nomoto"
        os.chdir(install_path+"/../../" + build_path) 

        # Run nomoto test
        subprocess.run("./tests")
       
    def test_2_dut_output(self, dut, proc_output):
        """
        Listen for a message published by dut and compare message to expected value.
        
        :param -
        :return dut [ros2 node] node to be tested (device under test)
        :return proc_output [ActiveIoHandler] data output of dut as shown in terminal (stdout)
        """
        # Get current functionname
        frame = inspect.currentframe()
        function_name = inspect.getframeinfo(frame).function

        # Publish data to dut
        self.publisher_ = self.node.create_publisher(FullSteering, 'nomoto/steering', 10)
        timer_period = 0.5  # seconds
        self.timer = self.node.create_timer(timer_period, self.timer_callback)
        
        # Read data of expected result
        EXPECTED_DATA_PATH = os.path.join(
            ament_index_python.get_package_prefix('shipmodel3dof'),
            'lib/shipmodel3dof',
            'expected_data.yaml')
        print(EXPECTED_DATA_PATH)
        with open(EXPECTED_DATA_PATH) as f:
            data = yaml.safe_load(f)
        
        expect_pos_x = data['msg']['pose']['position']['x']
        expect_pos_y = data['msg']['pose']['position']['y']
        expect_pos_z = data['msg']['pose']['position']['z']
        expect_orient_x = data['msg']['pose']['orientation']['x']
        expect_orient_y = data['msg']['pose']['orientation']['y']
        expect_orient_z = data['msg']['pose']['orientation']['z']
        expect_orient_w = data['msg']['pose']['orientation']['w']
        
        # Setup for listening to dut messages
        received_data = []
        sub = self.node.create_subscription(
            PoseStamped,
            '/nomoto/pose',
            lambda msg: received_data.append(msg),
            10
        )
        
        try:
            # Wait until the dut transmits a message over the ROS topic
            end_time = time.time() + 10
            
            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=0.1)
            
            if received_data == []:
                print(f"\n[{function_name}] !!!!!!!!!!!! No message received !!!!!!!!!!!!")
                test_data = ""
            else:
                msg_size = len(received_data) -1 # Number of messages received including zero(-1)
                test_data = received_data[msg_size] 
                print(test_data.pose.position.x)
            # test actual output for expected output
            self.assertLessEqual(test_data.pose.position.x, expect_pos_x, )
            self.assertLessEqual(test_data.pose.position.y, expect_pos_y)
            self.assertEqual(test_data.pose.position.z, expect_pos_z)
            #self.assertEqual(test_data.pose.orientation.x, expect_orient_x)
            #self.assertEqual(test_data.pose.orientation.y, expect_orient_y)
            #self.assertEqual(test_data.pose.orientation.z, expect_orient_z)
            #self.assertEqual(test_data.pose.orientation.w, expect_orient_w)

        finally:
            self.node.destroy_subscription(sub)
