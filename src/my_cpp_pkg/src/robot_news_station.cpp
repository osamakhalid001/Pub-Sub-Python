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

using namespace std::chrono_literals;

class RobotNewsStationNode : public rclcpp::Node
{
public:
  RobotNewsStationNode()
  : Node("robot_news_station"), robot_name_("R2D2")
  {
    publisher_ = this->create_publisher<example_interfaces::msg::String>("robot_news", 10);
    timer_ = this->create_wall_timer(
      0.5s, std::bind(&RobotNewsStationNode::publishNews, this));
    RCLCPP_INFO(this->get_logger(), "Robot News Station has been started");
  }

private:
  void publishNews()
  {
    auto msg = example_interfaces::msg::String();
    msg.data = std::string("Hi, this is ") + robot_name_ + \
      std::string(" from the robot news station.");
    publisher_->publish(msg);
  }

  std::string robot_name_;
  rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotNewsStationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
