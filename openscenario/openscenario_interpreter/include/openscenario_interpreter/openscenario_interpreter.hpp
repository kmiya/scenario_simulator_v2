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

#ifndef OPENSCENARIO_INTERPRETER__OPENSCENARIO_INTERPRETER_HPP_
#define OPENSCENARIO_INTERPRETER__OPENSCENARIO_INTERPRETER_HPP_

#include <openscenario_interpreter/utility/visibility.h>

#include <exception>
#include <junit_exporter/junit_exporter.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <memory>
#include <openscenario_interpreter/console/escape_sequence.hpp>
#include <openscenario_interpreter/syntax/openscenario.hpp>
#include <openscenario_interpreter/utility/verbose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <string>
#include <utility>

namespace openscenario_interpreter
{
class Interpreter : public rclcpp_lifecycle::LifecycleNode
{
  std::string intended_result;
  double local_frame_rate;
  double local_real_time_factor;
  std::string osc_path;
  std::string output_directory;

  Element script;

  std::shared_ptr<rclcpp::TimerBase> timer;

  junit_exporter::JunitExporter exporter;

  const junit_exporter::TestResult ERROR = junit_exporter::TestResult::ERROR;
  const junit_exporter::TestResult FAILURE = junit_exporter::TestResult::FAILURE;
  const junit_exporter::TestResult SUCCESS = junit_exporter::TestResult::SUCCESS;

  void report(const junit_exporter::TestResult &, const std::string &, const std::string & = "");

#define CATCH(TYPE)                       \
  catch (const TYPE & error)              \
  {                                       \
    if (intended_result == "error") {     \
      report(SUCCESS, #TYPE);             \
    } else {                              \
      report(ERROR, #TYPE, error.what()); \
    }                                     \
  }

  template <typename Thunk>
  void withExceptionHandler(Thunk && thunk)
  {
    using concealer::AutowareError;

    using openscenario_interpreter::ImplementationFault;
    using openscenario_interpreter::SemanticError;
    using openscenario_interpreter::SyntaxError;

    using InternalError = std::exception;

    try {
      return thunk();
    }

    catch (const SpecialAction<EXIT_SUCCESS> &) {
      if (intended_result == "success") {
        report(SUCCESS, "Success");
      } else {
        report(FAILURE, "UnintendedSuccess", "expected " + intended_result);
      }
    }

    catch (const SpecialAction<EXIT_FAILURE> &) {
      if (intended_result == "failure") {
        report(SUCCESS, "UnintendedFailure");
      } else {
        report(FAILURE, "Failure", "expected " + intended_result);
      }
    }

    CATCH(AutowareError)
    CATCH(ImplementationFault)
    CATCH(SemanticError)
    CATCH(SyntaxError)
    CATCH(InternalError)  // NOTE: THIS MUST BE LAST OF CATCH STATEMENTS.

    catch (...) { report(ERROR, "UnknownError", "An unknown exception has occurred"); }
  }

#undef CATCH

public:
  OPENSCENARIO_INTERPRETER_PUBLIC
  explicit Interpreter(const rclcpp::NodeOptions &);

  using Result = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  Result on_configure(const rclcpp_lifecycle::State &) override;

  Result on_activate(const rclcpp_lifecycle::State &) override;

  Result on_deactivate(const rclcpp_lifecycle::State &) override;

  Result on_cleanup(const rclcpp_lifecycle::State &) override;

  Result on_shutdown(const rclcpp_lifecycle::State &) override;

  Result on_error(const rclcpp_lifecycle::State &) override;
};
}  // namespace openscenario_interpreter

#endif  // OPENSCENARIO_INTERPRETER__OPENSCENARIO_INTERPRETER_HPP_
