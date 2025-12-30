// Copyright 2025 RbSCR
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <math.h>

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#define RAD2DEG(x) ((x)*180./M_PI)

class RplidarListener : public rclcpp::Node
{
public:
  RplidarListener() : Node("rplidar_listener")
  {
    auto param_desc_topic_name = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc_topic_name.description = "Topic name of Laserscan message to subscribe to.";
    topic_name_ = this->declare_parameter("topic_name", "scan", param_desc_topic_name);

    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                      topic_name_, 10,
                      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr scan)
                        {return this->scan_callback(scan);});
  }

private:
  void scan_callback(sensor_msgs::msg::LaserScan::ConstSharedPtr scan)
  {
    int count = scan->scan_time / scan->time_increment;

    RCLCPP_INFO(this->get_logger(), "New scan: %s[%d]", scan->header.frame_id.c_str(), count);
    RCLCPP_INFO(this->get_logger(), " scan time: [%f]", scan->scan_time);
    RCLCPP_INFO(this->get_logger(), " angle range: [%f, %f]",
                                    RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));
    RCLCPP_INFO(this->get_logger(), " angle increment: [%f]", RAD2DEG(scan->angle_increment));

    for (int i = 0; i < count; i++) {
      float degree = RAD2DEG(scan->angle_min + (scan->angle_increment * i));
      RCLCPP_INFO(this->get_logger(), " angle-distance-intensity : [%f, %f, %f]",
                                      degree, scan->ranges[i], scan->intensities[i]);
    }
  }

  std::string topic_name_;
  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::LaserScan>> subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<RplidarListener>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
