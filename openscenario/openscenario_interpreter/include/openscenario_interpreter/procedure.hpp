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

#ifndef OPENSCENARIO_INTERPRETER__PROCEDURE_HPP_
#define OPENSCENARIO_INTERPRETER__PROCEDURE_HPP_

#include <limits>
#include <memory>
#include <openscenario_interpreter/error.hpp>
#include <traffic_simulator/api/api.hpp>
#include <utility>

namespace openscenario_interpreter
{
extern traffic_simulator::API & connection;

template <typename... Ts>
decltype(auto) connect(Ts &&... xs)
{
  new (&connection) traffic_simulator::API(std::forward<decltype(xs)>(xs)...);
  return connection;
}

template <typename... Ts>
decltype(auto) getEntityStatus(Ts &&... xs)
try {
  return connection.getEntityStatus(std::forward<decltype(xs)>(xs)...);
} catch (const common::scenario_simulator_exception::SimulationError & error) {
  throw SemanticError(
    error.what(), ".\n", "Possible causes:\n",
    "  (1) The position of the corresponding entity is not specified by Teleport Action");
}

template <typename... Ts>
decltype(auto) getRelativePose(Ts &&... xs)
try {
  return connection.getRelativePose(std::forward<decltype(xs)>(xs)...);
} catch (const common::scenario_simulator_exception::SimulationError &) {
  geometry_msgs::msg::Pose result{};
  result.position.x = std::numeric_limits<double>::quiet_NaN();
  result.position.y = std::numeric_limits<double>::quiet_NaN();
  result.position.z = std::numeric_limits<double>::quiet_NaN();
  result.orientation.x = 0;
  result.orientation.y = 0;
  result.orientation.z = 0;
  result.orientation.w = 1;
  return result;
}

// template <typename... Ts>
// auto getDistanceAlongRoute(Ts&&... xs)
// {
//   if (const auto result {
//   connection->entity->getLongitudinalDistance(std::forward<decltype(xs)>(xs)...) })
//   {
//     return *result;
//   }
//   else
//   {
//     using value_type = typename std::decay<decltype(result)>::type::value_type;
//     return std::numeric_limits<value_type>::infinity();
//   }
// }

#define STRIP_OPTIONAL(IDENTIFIER, ALTERNATE)                                     \
  template <typename... Ts>                                                       \
  auto IDENTIFIER(Ts &&... xs)                                                    \
  {                                                                               \
    const auto result = connection.IDENTIFIER(std::forward<decltype(xs)>(xs)...); \
    if (result) {                                                                 \
      return result.get();                                                        \
    } else {                                                                      \
      using value_type = typename std::decay<decltype(result)>::type::value_type; \
      return ALTERNATE;                                                           \
    }                                                                             \
  }                                                                               \
  static_assert(true, "")

STRIP_OPTIONAL(getBoundingBoxDistance, static_cast<value_type>(0));
STRIP_OPTIONAL(getStandStillDuration, static_cast<value_type>(0));
STRIP_OPTIONAL(getTimeHeadway, std::numeric_limits<value_type>::quiet_NaN());

#undef STRIP_OPTIONAL

#define FORWARD_TO_SIMULATION_API(IDENTIFIER)                        \
  template <typename... Ts>                                          \
  decltype(auto) IDENTIFIER(Ts &&... xs)                             \
  {                                                                  \
    return connection.IDENTIFIER(std::forward<decltype(xs)>(xs)...); \
  }                                                                  \
  static_assert(true, "")

FORWARD_TO_SIMULATION_API(attachDetectionSensor);
FORWARD_TO_SIMULATION_API(attachLidarSensor);
FORWARD_TO_SIMULATION_API(checkCollision);
FORWARD_TO_SIMULATION_API(despawn);
FORWARD_TO_SIMULATION_API(engage);
FORWARD_TO_SIMULATION_API(getCurrentTime);
FORWARD_TO_SIMULATION_API(getTrafficLightArrow);
FORWARD_TO_SIMULATION_API(getTrafficLightColor);
FORWARD_TO_SIMULATION_API(initialize);
FORWARD_TO_SIMULATION_API(isInLanelet);
FORWARD_TO_SIMULATION_API(ready);
FORWARD_TO_SIMULATION_API(requestAcquirePosition);
FORWARD_TO_SIMULATION_API(requestAssignRoute);
FORWARD_TO_SIMULATION_API(requestLaneChange);
FORWARD_TO_SIMULATION_API(requestWalkStraight);
FORWARD_TO_SIMULATION_API(setEntityStatus);
FORWARD_TO_SIMULATION_API(setTargetSpeed);
FORWARD_TO_SIMULATION_API(setTrafficLightArrow);
FORWARD_TO_SIMULATION_API(setTrafficLightArrowPhase);
FORWARD_TO_SIMULATION_API(setTrafficLightColor);
FORWARD_TO_SIMULATION_API(setTrafficLightColorPhase);
FORWARD_TO_SIMULATION_API(spawn);
FORWARD_TO_SIMULATION_API(toMapPose);
FORWARD_TO_SIMULATION_API(updateFrame);

#undef FORWARD_TO_SIMULATION_API

#define RENAME(FROM, TO)                                       \
  template <typename... Ts>                                    \
  decltype(auto) TO(Ts &&... xs)                               \
  {                                                            \
    return connection.FROM(std::forward<decltype(xs)>(xs)...); \
  }                                                            \
  static_assert(true, "")

RENAME(reachPosition, isReachedPosition);
RENAME(setDriverModel, assignController);

#undef RENAME
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__PROCEDURE_HPP_
