#include <yaml-cpp/yaml.h>

#include "debugstream.hpp"
#include "teb_config.h"

using namespace teb_local_planner;

#define ADDCONFIG(a,b) decltype(tebconfig.a.b) b=a[#b].as<decltype(tebconfig.a.b)>();\
  tebconfig.a.b=b; \
  gDebugCol3(tebconfig.a.b);

inline bool ReadConfigXmlFile(TebConfig& tebconfig) {
  bool read_successful_flag=false;
  try {
    // Load the YAML file
    YAML::Node config = YAML::LoadFile( "/home/gxt_kt/Projects/teb/my_teb/teb_local_planner_arch/config.yaml");

    auto gxt = config["gxt"];
    ADDCONFIG(gxt,draw_arrow);
    ADDCONFIG(gxt,show_map);
    ADDCONFIG(gxt,show_button);

    // trajectory
    auto trajectory = config["trajectory"];
    ADDCONFIG(trajectory,teb_autosize);
    ADDCONFIG(trajectory,dt_ref)
    ADDCONFIG(trajectory,dt_hysteresis)
    ADDCONFIG(trajectory,min_samples)
    ADDCONFIG(trajectory,max_samples) 
    ADDCONFIG(trajectory,global_plan_overwrite_orientation) 
    ADDCONFIG(trajectory,allow_init_with_backwards_motion) 
    ADDCONFIG(trajectory,global_plan_viapoint_sep) 
    ADDCONFIG(trajectory,via_points_ordered) 
    ADDCONFIG(trajectory,max_global_plan_lookahead_dist) 
    ADDCONFIG(trajectory,global_plan_prune_distance) 
    ADDCONFIG(trajectory,exact_arc_length) 
    ADDCONFIG(trajectory,force_reinit_new_goal_dist) 
    ADDCONFIG(trajectory,force_reinit_new_goal_angular) 
    ADDCONFIG(trajectory,feasibility_check_no_poses) 
    ADDCONFIG(trajectory,publish_feedback) 
    ADDCONFIG(trajectory,min_resolution_collision_check_angular) 
    ADDCONFIG(trajectory,control_look_ahead_poses) 


    // 读取robot部分的数据
    auto robot = config["robot"];
    ADDCONFIG(robot,max_vel_x) 
    ADDCONFIG(robot,max_vel_x_backwards) 
    ADDCONFIG(robot,max_vel_y) 
    ADDCONFIG(robot,max_vel_theta) 
    ADDCONFIG(robot,acc_lim_x) 
    ADDCONFIG(robot,acc_lim_y) 
    ADDCONFIG(robot,acc_lim_theta) 
    ADDCONFIG(robot,min_turning_radius) 
    ADDCONFIG(robot,wheelbase) 
    ADDCONFIG(robot,cmd_angle_instead_rotvel) 
    ADDCONFIG(robot,is_footprint_dynamic) 

    // 读取goal_tolerance部分的数据
    auto goal_tolerance = config["goal_tolerance"];
    ADDCONFIG(goal_tolerance,xy_goal_tolerance)
    ADDCONFIG(goal_tolerance,yaw_goal_tolerance)
    ADDCONFIG(goal_tolerance,free_goal_vel)
    ADDCONFIG(goal_tolerance,complete_global_plan)

    // 读取obstacles部分的数据
    auto obstacles = config["obstacles"];
    ADDCONFIG(obstacles,min_obstacle_dist)
    ADDCONFIG(obstacles,inflation_dist)
    ADDCONFIG(obstacles,dynamic_obstacle_inflation_dist)
    ADDCONFIG(obstacles,include_dynamic_obstacles)
    ADDCONFIG(obstacles,include_costmap_obstacles)
    ADDCONFIG(obstacles,costmap_obstacles_behind_robot_dist)
    ADDCONFIG(obstacles,obstacle_poses_affected)
    ADDCONFIG(obstacles,legacy_obstacle_association)
    ADDCONFIG(obstacles,obstacle_association_force_inclusion_factor)
    ADDCONFIG(obstacles,obstacle_association_cutoff_factor)
    ADDCONFIG(obstacles,costmap_converter_plugin)
    ADDCONFIG(obstacles,costmap_converter_spin_thread)
    ADDCONFIG(obstacles,costmap_converter_rate)

    // 读取optim部分的数据
    auto optim = config["optim"];
    ADDCONFIG(optim,no_inner_iterations)
    ADDCONFIG(optim,no_outer_iterations)
    ADDCONFIG(optim,optimization_activate)
    ADDCONFIG(optim,optimization_verbose)
    ADDCONFIG(optim,penalty_epsilon)
    ADDCONFIG(optim,weight_max_vel_x)
    ADDCONFIG(optim,weight_max_vel_y)
    ADDCONFIG(optim,weight_max_vel_theta)
    ADDCONFIG(optim,weight_acc_lim_x)
    ADDCONFIG(optim,weight_acc_lim_y)
    ADDCONFIG(optim,weight_acc_lim_theta)
    ADDCONFIG(optim,weight_kinematics_nh)
    ADDCONFIG(optim,weight_kinematics_forward_drive)
    ADDCONFIG(optim,weight_kinematics_turning_radius)
    ADDCONFIG(optim,weight_optimaltime)
    ADDCONFIG(optim,weight_shortest_path)
    ADDCONFIG(optim,weight_obstacle)
    ADDCONFIG(optim,weight_inflation)
    ADDCONFIG(optim,weight_dynamic_obstacle)
    ADDCONFIG(optim,weight_dynamic_obstacle_inflation)
    ADDCONFIG(optim,weight_viapoint)
    ADDCONFIG(optim,weight_prefer_rotdir)

    ADDCONFIG(optim,weight_adapt_factor)
    ADDCONFIG(optim,obstacle_cost_exponent)

    // 读取hcp部分的数据
    auto hcp = config["hcp"];
    ADDCONFIG(hcp,enable_homotopy_class_planning)
    ADDCONFIG(hcp,enable_multithreading)
    ADDCONFIG(hcp,simple_exploration)
    ADDCONFIG(hcp,max_number_classes)
    ADDCONFIG(hcp,selection_cost_hysteresis)
    ADDCONFIG(hcp,selection_prefer_initial_plan)
    ADDCONFIG(hcp,selection_obst_cost_scale)
    ADDCONFIG(hcp,selection_viapoint_cost_scale)
    ADDCONFIG(hcp,selection_alternative_time_cost)

    ADDCONFIG(hcp,obstacle_keypoint_offset)
    ADDCONFIG(hcp,obstacle_heading_threshold)
    ADDCONFIG(hcp,roadmap_graph_no_samples)
    ADDCONFIG(hcp,roadmap_graph_area_width)
    ADDCONFIG(hcp,roadmap_graph_area_length_scale)
    ADDCONFIG(hcp,h_signature_prescaler)
    ADDCONFIG(hcp,h_signature_threshold)
    ADDCONFIG(hcp,switching_blocking_period)

    ADDCONFIG(hcp,viapoints_all_candidates)

    ADDCONFIG(hcp,visualize_hc_graph)
    ADDCONFIG(hcp,visualize_with_time_as_z_axis_scale)
    ADDCONFIG(hcp,delete_detours_backwards)
    ADDCONFIG(hcp,detours_orientation_tolerance)
    ADDCONFIG(hcp,length_start_orientation_vector)
    ADDCONFIG(hcp,max_ratio_detours_duration_best_duration)

    // // 读取recovery部分的数据
    auto recovery = config["recovery"];
    ADDCONFIG(recovery,shrink_horizon_backup)
    ADDCONFIG(recovery,shrink_horizon_min_duration)
    ADDCONFIG(recovery,oscillation_recovery)
    ADDCONFIG(recovery,oscillation_v_eps)
    ADDCONFIG(recovery,oscillation_omega_eps)
    ADDCONFIG(recovery,oscillation_recovery_min_duration)
    ADDCONFIG(recovery,oscillation_filter_duration)



    read_successful_flag=true;   
    gDebugCol1("READ CONFIG FILE SUCCESSFULLY!");
  } catch (const YAML::Exception& e) {
    // std::cerr << "Error while reading the YAML file: " << e.what() << std::endl;
    gDebugError("[GXT] : Error while reading the YAML file:")<< e.what();
  }

  if(read_successful_flag==false) {
    gDebugCol3("\n\n\n==========================================");
    gDebugCol3("[GXT] : Error while reading the YAML file!");
    gDebugCol3("[GXT] : Error while reading the YAML file!");
    gDebugError("[GXT] : Error while reading the YAML file!");
  }

  return 0;
}
