// Copyright 2026 rwh_beyond contributors
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#ifndef RWH_BEYOND_BT__PLUGINS__ACTION__MOVE_OBSERVATION_ACTION_HPP_
#define RWH_BEYOND_BT__PLUGINS__ACTION__MOVE_OBSERVATION_ACTION_HPP_

#include "rwh_beyond_bt/plugins/action/motion_action.hpp"

namespace rwh_beyond_bt
{
class MoveObservationAction : public MotionAction
{
public:
  using MotionAction::MotionAction;
  ~MoveObservationAction() override;
  static BT::PortsList providedPorts();

private:
  double default_z() override;
  void perform() override;
};
}  // namespace rwh_beyond_bt
#endif  // RWH_BEYOND_BT__PLUGINS__ACTION__MOVE_OBSERVATION_ACTION_HPP_
