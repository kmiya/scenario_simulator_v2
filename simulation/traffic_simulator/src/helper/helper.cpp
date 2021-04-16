// Copyright 2015-2020 Tier IV, Inc. All rights reserved.
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

#include <quaternion_operation/quaternion_operation.h>

#include <string>
#include <traffic_simulator/helper/helper.hpp>

namespace traffic_simulator
{
namespace helper
{
openscenario_msgs::msg::ActionStatus constructActionStatus(
  double linear_vel, double angular_vel, double linear_accel, double angular_accel)
{
  openscenario_msgs::msg::ActionStatus status;
  geometry_msgs::msg::Twist twist;
  twist.linear.x = linear_vel;
  twist.angular.z = angular_vel;
  geometry_msgs::msg::Accel accel;
  accel.linear.x = linear_accel;
  accel.angular.z = angular_accel;
  status.twist = twist;
  status.accel = accel;
  return status;
}

openscenario_msgs::msg::LaneletPose constructLaneletPose(
  std::int64_t lanelet_id, double s, double offset, double roll, double pitch, double yaw)
{
  openscenario_msgs::msg::LaneletPose lanelet_pose;
  lanelet_pose.lanelet_id = lanelet_id;
  lanelet_pose.s = s;
  lanelet_pose.offset = offset;
  lanelet_pose.rpy.x = roll;
  lanelet_pose.rpy.y = pitch;
  lanelet_pose.rpy.z = yaw;
  return lanelet_pose;
}

geometry_msgs::msg::Vector3 constructRPY(double roll, double pitch, double yaw)
{
  geometry_msgs::msg::Vector3 rpy;
  rpy.x = roll;
  rpy.y = pitch;
  rpy.z = yaw;
  return rpy;
}

geometry_msgs::msg::Vector3 constructRPYfronQuaternion(geometry_msgs::msg::Quaternion quaternion)
{
  return quaternion_operation::convertQuaternionToEulerAngle(quaternion);
}

geometry_msgs::msg::Pose constructPose(
  double x, double y, double z, double roll, double pitch, double yaw)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation =
    quaternion_operation::convertEulerAngleToQuaternion(constructRPY(roll, pitch, yaw));
  return pose;
}

std::ostream & operator<<(std::ostream & os, const openscenario_msgs::msg::LaneletPose & ll_pose)
{
  os << "lanelet id : " << ll_pose.lanelet_id << "\ns : " << ll_pose.s;
  return os;
}

const simulation_api_schema::DetectionSensorConfiguration constructDetectionSensorConfiguration(
  std::string entity, std::string topic_name, double update_duration)
{
  simulation_api_schema::DetectionSensorConfiguration configuration;
  configuration.set_entity(entity);
  configuration.set_topic_name(topic_name);
  configuration.set_update_duration(update_duration);
  return configuration;
}

const simulation_api_schema::LidarConfiguration constructLidarConfiguration(
  LidarType type, std::string entity, std::string topic_name, double horizontal_resolution)
{
  simulation_api_schema::LidarConfiguration configuration;
  configuration.set_horizontal_resolution(horizontal_resolution);
  configuration.set_topic_name(topic_name);
  configuration.set_entity(entity);
  switch (type) {
    case LidarType::VLP16:
      configuration.set_scan_duration(0.1);
      configuration.add_vertical_angles(-15.0 / 180.0 * M_PI);
      configuration.add_vertical_angles(-13.0 / 180.0 * M_PI);
      configuration.add_vertical_angles(-11.0 / 180.0 * M_PI);
      configuration.add_vertical_angles(-9.0 / 180.0 * M_PI);
      configuration.add_vertical_angles(-7.0 / 180.0 * M_PI);
      configuration.add_vertical_angles(-5.0 / 180.0 * M_PI);
      configuration.add_vertical_angles(-3.0 / 180.0 * M_PI);
      configuration.add_vertical_angles(-1.0 / 180.0 * M_PI);
      configuration.add_vertical_angles(1.0 / 180.0 * M_PI);
      configuration.add_vertical_angles(3.0 / 180.0 * M_PI);
      configuration.add_vertical_angles(5.0 / 180.0 * M_PI);
      configuration.add_vertical_angles(7.0 / 180.0 * M_PI);
      configuration.add_vertical_angles(9.0 / 180.0 * M_PI);
      configuration.add_vertical_angles(11.0 / 180.0 * M_PI);
      configuration.add_vertical_angles(13.0 / 180.0 * M_PI);
      configuration.add_vertical_angles(15.0 / 180.0 * M_PI);
      break;
    case LidarType::VLP32:
      configuration.set_scan_duration(0.1);
      configuration.add_vertical_angles(-15.0 / 180.0 * M_PI);
      configuration.add_vertical_angles(-13.66666667 / 180.0 * M_PI);
      configuration.add_vertical_angles(-12.33333333 / 180.0 * M_PI);
      configuration.add_vertical_angles(-11.0 / 180.0 * M_PI);
      configuration.add_vertical_angles(-9.66666667 / 180.0 * M_PI);
      configuration.add_vertical_angles(-8.33333333 / 180.0 * M_PI);
      configuration.add_vertical_angles(-7.0 / 180.0 * M_PI);
      configuration.add_vertical_angles(-5.66666667 / 180 * M_PI);
      configuration.add_vertical_angles(-4.33333333 / 180.0 * M_PI);
      configuration.add_vertical_angles(-3.0 / 180.0 * M_PI);
      configuration.add_vertical_angles(-1.66666667 / 180.0 * M_PI);
      configuration.add_vertical_angles(-0.33333333 / 180.0 * M_PI);
      configuration.add_vertical_angles(1.0 / 180.0 * M_PI);
      configuration.add_vertical_angles(2.33333333 / 180.0 * M_PI);
      configuration.add_vertical_angles(3.66666667 / 180.0 * M_PI);
      configuration.add_vertical_angles(5.0 / 180.0 * M_PI);
      configuration.add_vertical_angles(6.33333333 / 180.0 * M_PI);
      configuration.add_vertical_angles(7.66666667 / 180.0 * M_PI);
      configuration.add_vertical_angles(9.0 / 180.0 * M_PI);
      configuration.add_vertical_angles(10.33333333 / 180.0 * M_PI);
      configuration.add_vertical_angles(11.66666667 / 180.0 * M_PI);
      configuration.add_vertical_angles(13.0 / 180.0 * M_PI);
      configuration.add_vertical_angles(14.33333333 / 180.0 * M_PI);
      configuration.add_vertical_angles(15.66666667 / 180.0 * M_PI);
      configuration.add_vertical_angles(17.0 / 180.0 * M_PI);
      configuration.add_vertical_angles(18.33333333 / 180.0 * M_PI);
      configuration.add_vertical_angles(19.66666667 / 180.0 * M_PI);
      configuration.add_vertical_angles(21.0 / 180.0 * M_PI);
      configuration.add_vertical_angles(22.33333333 / 180.0 * M_PI);
      configuration.add_vertical_angles(23.66666667 / 180.0 * M_PI);
      configuration.add_vertical_angles(25.0 / 180.0 * M_PI);
      break;
  }
  return configuration;
}

}  // namespace helper
}  // namespace traffic_simulator