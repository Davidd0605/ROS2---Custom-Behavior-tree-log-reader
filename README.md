# ROS2---Custom-Behavior-tree-log-reader

- Custom C++ subscriber that reads from the /behavior_tree_log topic and converts the input into boolean flags that represent the feasibility of the current goal.

**Input topic**
/behavior_tree_log

**Output topic**
/goal_reachable

## How to use:
  - Clone repository in the src directory in your ROS2 workspace
  - Do colcon build
  - In a separate terminal run: ros2 run behavior_tree_monitor behavior_tree_monitor
  - Output can be accessed using the /goal_reachable topic.

## Alternatively
  - Go in the unity_slam_whatever package->launch->go in the slam py file
  - Add in launch description:

```python
Node(
    package='behavior_tree_monitor',
    executable='behavior_tree_monitor',
    name='behavior_tree_monitor',
    output='screen',
    parameters=[{'use_sim_time': True}]
)

