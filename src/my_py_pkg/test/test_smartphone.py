#!/usr/bin/env python3
# Copyright 2025 Your Name or Organization
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import unittest
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String
from unittest.mock import patch
from my_py_pkg.smartphone import SmartphoneNode  # Adjust import based on your package structure


class TestSmartphoneNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        """Initialize rclpy once for all tests."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shutdown rclpy after all tests."""
        rclpy.shutdown()

    def setUp(self):
        """Create a test node and SmartphoneNode instance for each test."""
        self.node = Node("test_node")
        self.smartphone_node = SmartphoneNode()
        self.publisher = self.node.create_publisher(String, "robot_news", 10)

    def tearDown(self):
        """Destroy nodes after each test."""
        self.node.destroy_node()
        self.smartphone_node.destroy_node()

    def test_node_initialization(self):
        """Test that SmartphoneNode initializes with correct name and subscription."""
        self.assertEqual(self.smartphone_node.get_name(), "smartphone")
        # subscriptions = self.smartphone_node.get_subscription_names()
        # self.assertIn("robot_news", subscriptions)
        self.assertIsNotNone(self.smartphone_node.subscriber_)
        self.assertEqual(self.smartphone_node.subscriber_.topic_name, "/robot_news")

    @patch.object(SmartphoneNode, 'get_logger')
    def test_callback_robot_news(self, mock_get_logger):
        """Test that callback_robot_news logs the received message."""
        test_message = String()
        test_message.data = "Test robot news"

        # Publish the test message
        self.publisher.publish(test_message)

        # Spin once to process callbacks
        rclpy.spin_once(self.smartphone_node, timeout_sec=0.1)

        # Verify that the logger was called with the correct message
        mock_get_logger.return_value.info.assert_called_once_with("Test robot news")

if __name__ == "__main__":
    unittest.main()
