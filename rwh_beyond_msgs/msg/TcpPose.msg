# One TCP target shared through a single BehaviorTree blackboard entry.
# Position: metres. Orientation: quaternion x/y/z/w in frame_id.
string frame_id "base_link"
geometry_msgs/Pose pose
# Interpret only position as an offset along frame_id axes.
bool relative false
# If true, ignore pose.orientation and retain the current TCP orientation.
# Otherwise a finite, nonzero quaternion is required.
bool keep_current_orientation false
