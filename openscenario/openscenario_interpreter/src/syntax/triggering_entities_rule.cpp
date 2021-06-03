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

#include <openscenario_interpreter/error.hpp>
#include <openscenario_interpreter/syntax/triggering_entities_rule.hpp>

namespace openscenario_interpreter
{
inline namespace syntax
{
std::istream & operator>>(std::istream & is, TriggeringEntitiesRule & rule)
{
  std::string buffer;

  is >> buffer;

#define BOILERPLATE(IDENTIFIER)                      \
  if (buffer == #IDENTIFIER) {                       \
    rule.value = TriggeringEntitiesRule::IDENTIFIER; \
    return is;                                       \
  }                                                  \
  static_assert(true, "")

  BOILERPLATE(all);
  BOILERPLATE(any);

#undef BOILERPLATE

  throw UNEXPECTED_ENUMERATION_VALUE_SPECIFIED(TriggeringEntitiesRule, buffer);
}

std::ostream & operator<<(std::ostream & os, const TriggeringEntitiesRule & datum)
{
#define BOILERPLATE(ID)            \
  case TriggeringEntitiesRule::ID: \
    return os << #ID;

  switch (datum) {
    BOILERPLATE(all);
    BOILERPLATE(any);

    default:
      throw UNEXPECTED_ENUMERATION_VALUE_ASSIGNED(TriggeringEntitiesRule, datum);
  }

#undef BOILERPLATE
}
}  // namespace syntax
}  // namespace openscenario_interpreter