// Copyright 2024 SYED OSAMA KHALID
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"

using namespace std::placeholders;

class SmartphoneNode : public rclcpp::Node
{
public:
  SmartphoneNode()
  : Node("smartphone")
  {
    subscriber_ = this->create_subscription<example_interfaces::msg::String>(
      "robot_news", 10,
      std::bind(&SmartphoneNode::callbackRobotNews, this, _1));
    RCLCPP_INFO(this->get_logger(), "Smartphone has been started.");
  }

private:
  void callbackRobotNews(const example_interfaces::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "%s", msg->data.c_str());
  }

  rclcpp::Subscription<example_interfaces::msg::String>::SharedPtr subscriber_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SmartphoneNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
