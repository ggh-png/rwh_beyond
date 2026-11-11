# Condition plugins

No condition plugin is implemented yet. Add declarations under
`include/rwh_beyond_bt/plugins/condition/`, matching implementations here, and
register each plugin with its own `BT_REGISTER_NODES` entry point and CMake target.
Update the compatibility loader if existing configurations should load it.
