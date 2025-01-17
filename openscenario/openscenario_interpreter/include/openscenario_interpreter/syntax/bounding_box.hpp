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

#ifndef OPENSCENARIO_INTERPRETER__SYNTAX__BOUNDING_BOX_HPP_
#define OPENSCENARIO_INTERPRETER__SYNTAX__BOUNDING_BOX_HPP_

#include <openscenario_interpreter/syntax/center.hpp>
#include <openscenario_interpreter/syntax/dimensions.hpp>
#include <openscenario_msgs/msg/bounding_box.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
/* ---- BoundingBox ------------------------------------------------------------
 *
 *  <xsd:complexType name="BoundingBox">
 *    <xsd:all>
 *      <xsd:element name="Center" type="Center"/>
 *      <xsd:element name="Dimensions" type="Dimensions"/>
 *    </xsd:all>
 *  </xsd:complexType>
 *
 * -------------------------------------------------------------------------- */
struct BoundingBox
{
  const Center center;

  const Dimensions dimensions;

  BoundingBox() = default;

  template <typename Node, typename Scope>
  explicit BoundingBox(const Node & node, Scope & scope)
  : center(readElement<Center>("Center", node, scope)),
    dimensions(readElement<Dimensions>("Dimensions", node, scope))
  {
  }

  explicit operator openscenario_msgs::msg::BoundingBox() const
  {
    openscenario_msgs::msg::BoundingBox bounding_box;
    {
      bounding_box.center = static_cast<geometry_msgs::msg::Point>(center);
      bounding_box.dimensions = static_cast<geometry_msgs::msg::Vector3>(dimensions);
    }

    return bounding_box;
  }
};

std::ostream & operator<<(std::ostream &, const BoundingBox &);
}  // namespace syntax
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__SYNTAX__BOUNDING_BOX_HPP_
