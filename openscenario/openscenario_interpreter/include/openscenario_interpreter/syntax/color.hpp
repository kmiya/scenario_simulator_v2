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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__COLOR_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__COLOR_HPP_

#include <boost/optional.hpp>
#include <iostream>
#include <traffic_simulator/traffic_lights/traffic_light_state.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
struct Color
{
  enum value_type {
    noColor,
    green,
    red,
    yellow,
  } value;

  constexpr Color(value_type value = noColor) : value(value) {}

  constexpr operator value_type() const noexcept { return value; }

  operator traffic_simulator::TrafficLightColor() const
  {
    switch (value) {
      case noColor:
        return traffic_simulator::TrafficLightColor::NONE;

      case green:
        return traffic_simulator::TrafficLightColor::GREEN;

      case red:
        return traffic_simulator::TrafficLightColor::RED;

      case yellow:
        return traffic_simulator::TrafficLightColor::YELLOW;

      default:
        throw UNEXPECTED_ENUMERATION_VALUE_ASSIGNED(Color, *this);
    }
  }
};

static_assert(std::is_trivially_copy_constructible<Color>::value, "");
static_assert(std::is_trivially_copy_assignable<Color>::value, "");
static_assert(std::is_standard_layout<Color>::value, "");

std::istream & operator>>(std::istream &, Color &);
std::istream & operator>>(std::istream &, boost::optional<Color> &);

std::ostream & operator<<(std::ostream &, const Color &);
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__COLOR_HPP_
