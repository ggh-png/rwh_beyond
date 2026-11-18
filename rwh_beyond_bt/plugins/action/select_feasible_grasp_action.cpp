// Copyright 2026 rwh_beyond contributors
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#include "rwh_beyond_bt/plugins/action/select_feasible_grasp_action.hpp"

namespace rwh_beyond_bt
{
SelectFeasibleGraspAction::~SelectFeasibleGraspAction() {stop();}

BT::PortsList SelectFeasibleGraspAction::providedPorts()
{
  return motion_ports({"fusion", "capture", "mask", "cloud", "candidates", "use_estimated_width",
             "pregrasp_distance", "lift_distance", "selected_plan", "pregrasp", "grasp_pose",
             "grasp_width", "close_width"});
}

void SelectFeasibleGraspAction::prepare()
{
  selected_.reset();
  fusion_.reset();
  if (config().input_ports.count("fusion")) {fusion_ = required<Fusion>("fusion");}
  capture_ = required<Observation>("capture");
  mask_ = required<sensor_msgs::msg::Image>("mask");
  cloud_ = required<sensor_msgs::msg::PointCloud2>("cloud");
  candidates_ = required<std::vector<rwh_beyond_msgs::msg::GraspCandidate>>("candidates");
  estimated_ = required<bool>("use_estimated_width");
  pre_ = scalar("pregrasp_distance", context_->number("pregrasp_distance", .1, .02, .2), .02,
      .2);
  lift_ = scalar("lift_distance", context_->number("lift_distance", .08, .065, .2), .065, .2);
}

void SelectFeasibleGraspAction::perform()
{
  selected_ = context_->select(capture_, mask_, cloud_, candidates_, estimated_, pre_, lift_,
      cancel_, fusion_);
}

void SelectFeasibleGraspAction::publish_outputs()
{
  if (selected_) {
    setOutput("selected_plan", selected_);
    TcpPose pose; pose.frame_id = context_->base; pose.pose = selected_->pregrasp;
    setOutput("pregrasp", pose);
    pose.pose = selected_->grasp; setOutput("grasp_pose", pose);
    setOutput("grasp_width", selected_->observed_width);
    setOutput("close_width", selected_->close_width);
  }
}
}  // namespace rwh_beyond_bt
