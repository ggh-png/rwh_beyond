#include "rwh_beyond_bt/register_nodes.hpp"

BT_REGISTER_NODES(factory)
{
  rwh_beyond_bt::register_move_home(factory);
  rwh_beyond_bt::register_move_tcp_up(factory);
  rwh_beyond_bt::register_close_gripper(factory);
  rwh_beyond_bt::register_open_gripper(factory);
  rwh_beyond_bt::register_set_gripper(factory);
  rwh_beyond_bt::register_move_tcp(factory);
  rwh_beyond_bt::register_make_tcp_pose(factory);
}
