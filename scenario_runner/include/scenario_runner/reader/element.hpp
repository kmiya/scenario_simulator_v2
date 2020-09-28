// Copyright 2015-2020 Autoware Foundation. All rights reserved.
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

#ifndef SCENARIO_RUNNER__READER__ELEMENT_HPP_
#define SCENARIO_RUNNER__READER__ELEMENT_HPP_

#include <scenario_runner/iterator/size.hpp>
#include <scenario_runner/object.hpp>
#include <scenario_runner/type_traits/if_not_default_constructible.hpp>
#include <scenario_runner/utility/pugi_extension.hpp>

#include <iterator>
#include <limits>
#include <string>
#include <utility>

namespace scenario_runner
{
inline namespace reader
{
constexpr auto unbounded {
  std::numeric_limits<
    typename std::iterator_traits<
      typename pugi::xml_node::iterator
    >::difference_type
  >::max()
};

template<typename T, typename Node, typename ... Ts>
auto readElement(const std::string & name, const Node & parent, Ts && ... xs)
{
  if (const auto child {parent.child(name.c_str())}) {
    return T {child, std::forward<decltype(xs)>(xs)...};
  } else {
    return IfNotDefaultConstructible<T>::error(parent.name(), name);
  }
}

template<typename Node, typename Callee>
void callWithElements(
  const Node & parent,
  const std::string & name,
  typename std::iterator_traits<typename Node::iterator>::difference_type min_occurs,
  typename std::iterator_traits<typename Node::iterator>::difference_type max_occurs,
  Callee && call_with)
{
  const auto children {parent.children(name.c_str())};

  // NOTE ament_uncrustify says this malformed indentation is beautiful, and forced us to do so.

  if (const auto size {iterator::size(children)}) {
    if (min_occurs != 0 && size < min_occurs) {
      std::stringstream ss {};
      ss << parent.name() <<
        " requires class " <<
        name <<
        " at least " <<
        min_occurs <<
        " element" <<
      (1 < min_occurs ? "s" : "") <<
        ", but " <<
        size <<
        " element" <<
      (1 < size ? "s" : "") <<
        " specified";
      throw SyntaxError {ss.str()};
    } else if (max_occurs < size) {
      std::stringstream ss {};
      ss << parent.name() <<
        " requires class " <<
        name <<
        " at most " <<
        max_occurs <<
        " element" <<
      (1 < max_occurs ? "s" : "") <<
        ", but " <<
        size <<
        " element" <<
      (1 < size ? "s" : "") <<
        " specified";
      throw SyntaxError {ss.str()};
    } else {
      for (const auto & child : children) {
        call_with(child);
      }
    }
  } else if (min_occurs != 0) {
    std::stringstream ss {};
    ss << parent.name() <<
      " requires class " <<
      name << " at least " <<
      min_occurs <<
      " element" <<
    (1 < min_occurs ? "s" : "") <<
      ", but there is no specification";
    throw SyntaxError {ss.str()};
  }
}

template<typename Callee>
decltype(auto) callWithElement(const pugi::xml_node & parent, const std::string & name,
  Callee && call_with)
{
  return callWithElements(parent, name, 1, 1, std::forward<decltype(call_with)>(call_with));
}

  #define THROW_UNSUPPORTED_ERROR(PARENT) \
  [&](auto && child) \
  { \
    std::stringstream ss {}; \
    ss << "given class \'" << child.name() << "\' (element of class \'" << PARENT.name() << \
      "\') is valid OpenSCENARIO element, but is not supported"; \
    throw SyntaxError {ss.str()}; \
  }
}
}  // namespace scenario_runner

#endif  // SCENARIO_RUNNER__READER__ELEMENT_HPP_