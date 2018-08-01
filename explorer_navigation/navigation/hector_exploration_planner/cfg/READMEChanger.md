#! /usr/bin/env python
PACKAGE='hector_exploration_planner'

from dynamic_reconfigure.parameter_generator_catkin import *

gen = ParameterGenerator()

gen.add("security_constant", double_t, 0, "security_constant", 0.5, 0.0, 10000.0)

#! /usr/bin/env python安全常数

gen.add("min_obstacle_dist", int_t, 0, "min_obstacle_dist", 10, 0, 1000)
##最小的障碍区

gen.add("plan_in_unknown", bool_t, 0, "plan_in_unknown", False)
##是否在未知区域进行探索　False True

gen.add("explore_close_to_path", bool_t, 0, "explore_close_to_path", True)
##接近路径探索

gen.add("use_inflated_obstacles", bool_t, 0, "use_inflated_obstacle", True)
#是否使用膨胀半径

gen.add("goal_angle_penalty", int_t, 0, "goal_angle_penalty", 50, 0, 10000)

gen.add("min_frontier_size", int_t, 0, "min_frontier_size", 5, 0, 1000)
##最小边界尺寸

gen.add("dist_for_goal_reached", double_t, 0, "dist_for_goal_reached", 0.25, 0.0, 10.0)
##对于目标达成的距离

gen.add("same_frontier_distance", double_t, 0, "same_frontier_distance", 0.25, 0.0, 10.0)
##相同的边界的距离

gen.add("obstacle_cutoff_distance", double_t, 0, "obstacle_cutoff_distance", 1.0, 0.0, 100.0)
##截止距离的障碍

gen.add("use_observation_pose_calcullation", bool_t, 0, "use observation pose calculation", True)
##使用观测的姿态计算

gen.add("observation_pose_desired_dist", double_t, 0, "Observation pose desired distance", 0.5, 0.0, 10.0)
##观察构成所需的距离

gen.add("observation_pose_allowed_angle", double_t, 0, "allowed angle for observation pose adjustment [deg]", 135.0, 0.0, 360.0)
##observation pose allowed angle

gen.add("close_to_path_target_distance", double_t, 0, "close to path exploration target distance", 1.2, 0.0, 10.0)
##接近路径目标距离

exit(gen.generate(PACKAGE, "hector_exploration_planner", "ExplorationPlanner"))
