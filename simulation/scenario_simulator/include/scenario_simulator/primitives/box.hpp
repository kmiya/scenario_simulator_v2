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


#ifndef SCENARIO_SIMULATOR__PRIMITIVES__BOX_HPP_
#define SCENARIO_SIMULATOR__PRIMITIVES__BOX_HPP_

#include <scenario_simulator/primitives/primitive.hpp>
#include <simulation_api_schema.pb.h>

namespace scenario_simulator
{
namespace primitives
{
class Box : public Primitive
{
public:
  explicit Box(float depth, float width, float height, geometry_msgs::msg::Pose pose);
  ~Box() = default;
  const float depth;
  const float width;
  const float height;
};
}  // namespace primitives
}  // namespace scenario_simulator

#endif  // SCENARIO_SIMULATOR__PRIMITIVES__BOX_HPP_