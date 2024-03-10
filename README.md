# warehouse_project

### simulation

```
ros2 launch localization_server localization.launch.py
ros2 launch path_planner_server pathplanner.launch.py
python3 ~/ros2_ws/src/nav2_apps/scripts/move_shelf_to_ship.py
```

### Real robot

```
ros2 launch localization_server localization.launch.py use_sim_time:=False map_file:=warehouse_map_real.yaml
ros2 launch path_planner_server pathplanner.launch.py use_sim_time:=False keepout_map_file:=warehouse_map_real_keepout.yaml is_elevator_topic_string:=True cmd_vel_topic:=/cmd_vel
python3 ~/ros2_ws/src/nav2_apps/scripts/move_shelf_to_ship.py --real_robot
```