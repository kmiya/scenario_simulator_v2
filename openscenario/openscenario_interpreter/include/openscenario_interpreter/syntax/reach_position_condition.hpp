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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__REACH_POSITION_CONDITION_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__REACH_POSITION_CONDITION_HPP_

#include <openscenario_interpreter/procedure.hpp>
#include <openscenario_interpreter/syntax/position.hpp>
#include <openscenario_interpreter/syntax/triggering_entities.hpp>
#include <traffic_simulator/helper/helper.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- ReachPositionCondition -------------------------------------------------
 *
 *  <xsd:complexType name="ReachPositionCondition">
 *    <xsd:all>
 *      <xsd:element name="Position" type="Position"/>
 *    </xsd:all>
 *    <xsd:attribute name="tolerance" type="Double" use="required"/>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct ReachPositionCondition
{
  const Double tolerance;

  const Position position;

  const Rule compare;

  const TriggeringEntities triggering_entities;

  std::vector<Double> last_checked_values;  // for description

  template <typename Node, typename Scope>
  explicit ReachPositionCondition(
    const Node & node, Scope & outer_scope, const TriggeringEntities & triggering_entities)
  : tolerance(readAttribute<Double>("tolerance", node, outer_scope)),
    position(readElement<Position>("Position", node, outer_scope)),
    compare(Rule::lessThan),
    triggering_entities(triggering_entities),
    last_checked_values(triggering_entities.entity_refs.size(), Double::nan())
  {
  }

  bool operator()(const WorldPosition & world_position, const EntityRef & triggering_entity) const
  {
    return isReachedPosition(
      triggering_entity, static_cast<geometry_msgs::msg::Pose>(world_position), tolerance);
  }

  bool operator()(const RelativeWorldPosition &, const EntityRef &) const
  {
    throw UNSUPPORTED_SETTING_DETECTED(ReachPositionCondition, position.type().name());
  }

  bool operator()(const LanePosition & lane_position, const EntityRef & triggering_entity) const
  {
    return isReachedPosition(
      triggering_entity, static_cast<openscenario_msgs::msg::LaneletPose>(lane_position),
      tolerance);
  }

  auto description() const
  {
    std::stringstream description;

    description << triggering_entities.description() << "'s distance to given position = ";

    print_to(description, last_checked_values);

    description << " " << compare << " " << tolerance << "?";

    return description.str();
  }

  auto evaluate()
  {
    auto distance = [&](const EntityRef & name) {
      const auto pose = getRelativePose(name, static_cast<geometry_msgs::msg::Pose>(position));
      return std::hypot(pose.position.x, pose.position.y);
    };

    last_checked_values.clear();

    return asBoolean(triggering_entities.apply([&](const auto & triggering_entity) {
      last_checked_values.push_back(distance(triggering_entity));
      return apply<bool>(*this, position, triggering_entity);
    }));
  }
};
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__REACH_POSITION_CONDITION_HPP_
