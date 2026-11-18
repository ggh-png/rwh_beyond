// Copyright 2026 rwh_beyond contributors
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#ifndef RWH_BEYOND_BT__PLUGINS__ACTION__MOTION_ACTION_HPP_
#define RWH_BEYOND_BT__PLUGINS__ACTION__MOTION_ACTION_HPP_

#include <initializer_list>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"
#include "rwh_beyond_bt/motion_context.hpp"
#include "rwh_beyond_bt/tcp_pose.hpp"

namespace rwh_beyond_bt
{
BT::PortsList motion_ports(std::initializer_list<const char *> names);

class MotionAction : public BT::StatefulActionNode
{
public:
  MotionAction(const std::string & name, const BT::NodeConfiguration & config);
  ~MotionAction() override;
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

protected:
  virtual void prepare() {}
  virtual void perform() = 0;
  virtual void publish_outputs() {}
  virtual double default_z() {return 0;}
  virtual double minimum_z() {return -.3;}
  // Derived destructors must stop the worker before virtual dispatch becomes invalid.
  void stop();
  void prepare_plan();
  void phase(GraspPlan::Phase expected);
  State home_state(const State & state);
  bool at_home();
  double scalar(const std::string & port, double fallback, double low, double high);

  template<class T> T required(const std::string & port)
  {
    auto result = getInput<T>(port);
    if (!result) {throw std::invalid_argument("Missing/invalid " + port + ": " + result.error());}
    return result.value();
  }

  std::shared_ptr<MotionContext> context_;
  Cancel cancel_;
  Plan input_plan_;
  std::vector<double> home_, tilt_;
  double x_{0}, y_{0}, z_{0}, yaw_{0}, settle_{.5}, opening_{.10}, duration_{3};

private:
  BT::NodeStatus failure(const std::exception & error);
  std::future<void> future_;
};

}  // namespace rwh_beyond_bt
#endif  // RWH_BEYOND_BT__PLUGINS__ACTION__MOTION_ACTION_HPP_
