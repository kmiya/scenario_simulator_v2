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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__ACQUIRE_POSITION_ACTION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__ACQUIRE_POSITION_ACTION_HPP_

#include <openscenario_interpreter/procedure.hpp>
#include <openscenario_interpreter/syntax/position.hpp>
#include <openscenario_interpreter/syntax/string.hpp>
#include <traffic_simulator/helper/helper.hpp>
#include <unordered_map>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- AcquirePositionAction --------------------------------------------------
 *
 *  <xsd:complexType name="AcquirePositionAction">
 *    <xsd:all>
 *      <xsd:element name="Position" type="Position"/>
 *    </xsd:all>
 *  </xsd:complexType>
 *
 *  TODO REMOVE EXTENSION
 *
 * -------------------------------------------------------------------------- */
struct AcquirePositionAction : private Scope
{
  const Position position;

  template <typename Node>
  explicit AcquirePositionAction(const Node & node, Scope & outer_scope)
  : Scope(outer_scope), position(readElement<Position>("Position", node, localScope()))
  {
  }

  auto operator()(const Scope::Actor & actor) const
  {
    if (position.is<LanePosition>()) {
      requestAcquirePosition(
        actor, static_cast<openscenario_msgs::msg::LaneletPose>(position.as<LanePosition>()));
    } else {
      throw UNSUPPORTED_SETTING_DETECTED(AcquirePositionAction, position.type().name());
    }
  }

  std::unordered_map<String, Boolean> accomplishments;

  auto reset()
  {
    accomplishments.clear();
    for (const auto & actor : actors) {
      accomplishments.emplace(actor, false);
    }
  }

  auto start()
  {
    reset();

    for (const auto & actor : actors) {
      (*this)(actor);
    }

    return unspecified;
  }

  auto check(const String & actor)
  {
    if (position.is<LanePosition>()) {
      return isReachedPosition(
        actor, static_cast<openscenario_msgs::msg::LaneletPose>(position.as<LanePosition>()), 1.0);
    } else {
      throw UNSUPPORTED_SETTING_DETECTED(AcquirePositionAction, position.type().name());
    }
  }

  auto update()
  {
    for (auto && each : accomplishments) {
      each.second = each.second or check(each.first);
    }
  }

  static constexpr auto accomplished() { return true; }

  static constexpr bool endsImmediately() { return true; };
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__ACQUIRE_POSITION_ACTION_HPP_
