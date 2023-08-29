#include <boost/thread/once.hpp>
#include <boost/thread/thread.hpp>
#include <limits>
#include <map>
#include <memory>

#include "../inc/optimal_planner.h"

#include "debugstream.hpp"

namespace teb_local_planner {

// ============== Implementation ===================

TebOptimalPlanner::TebOptimalPlanner()
    : cfg_(NULL),
      obstacles_(NULL),
      via_points_(NULL),
      cost_(HUGE_VAL),
      prefer_rotdir_(RotType::none),
      robot_model_(new PointRobotFootprint()),
      initialized_(false),
      optimized_(false) {}

TebOptimalPlanner::TebOptimalPlanner(const TebConfig& cfg,
                                     ObstContainer* obstacles,
                                     RobotFootprintModelPtr robot_model,
                                     TebVisualizationPtr visual,
                                     const ViaPointContainer* via_points) {
  initialize(cfg, obstacles, robot_model, visual, via_points);
}

TebOptimalPlanner::~TebOptimalPlanner() {
  clearGraph();
  // free dynamically allocated memory
  // if (optimizer_)
  //  g2o::Factory::destroy();
  // g2o::OptimizationAlgorithmFactory::destroy();
  // g2o::HyperGraphActionLibrary::destroy();
}

void TebOptimalPlanner::initialize(const TebConfig& cfg,
                                   ObstContainer* obstacles,
                                   RobotFootprintModelPtr robot_model,
                                   TebVisualizationPtr visual,
                                   const ViaPointContainer* via_points) {
  // init optimizer (set solver and block ordering settings)
  optimizer_ = initOptimizer(); // NOTE: init g2o

  cfg_ = &cfg;
  obstacles_ = obstacles;
  robot_model_ = robot_model;
  via_points_ = via_points;
  cost_ = HUGE_VAL;
  prefer_rotdir_ = RotType::none;
  setVisualization(visual);

  vel_start_.first = true;
  vel_start_.second.linear.x() = 0;
  vel_start_.second.linear.y() = 0;
  vel_start_.second.angular.z() = 0;

  vel_goal_.first = true;
  vel_goal_.second.linear.x() = 0;
  vel_goal_.second.linear.y() = 0;
  vel_goal_.second.angular.z() = 0;
  initialized_ = true;
}

void TebOptimalPlanner::setVisualization(TebVisualizationPtr visualization) {
  visualization_ = visualization;
}

void TebOptimalPlanner::visualize() {
  if (!visualization_) return;

  visualization_->publishLocalPlanAndPoses(teb_);

  if (cfg_->trajectory.publish_feedback)
    visualization_->publishFeedbackMessage(*this, *obstacles_);
}

/*
 * registers custom vertices and edges in g2o framework
 */
void TebOptimalPlanner::registerG2OTypes() {
  g2o::Factory* factory = g2o::Factory::instance();
  // factory->registerType(
  //     "VERTEX_POSE",
  //     std::make_shared<g2o::HyperGraphElementCreator<VertexPose>>());
  // factory->registerType(
  //     "VERTEX_TIMEDIFF",
  //     std::make_shared<g2o::HyperGraphElementCreator<VertexTimeDiff>>());

  // factory->registerType("EDGE_TIME_OPTIMAL",
  // std::make_shared<g2o::HyperGraphElementCreator<EdgeTimeOptimal>>());
  // factory->registerType("EDGE_SHORTEST_PATH",
  // std::make_shared<g2o::HyperGraphElementCreator<EdgeShortestPath>>());
  // factory->registerType("EDGE_VELOCITY",
  // std::make_shared<g2o::HyperGraphElementCreator<EdgeVelocity>>());
  // factory->registerType("EDGE_VELOCITY_HOLONOMIC",
  // std::make_shared<g2o::HyperGraphElementCreator<EdgeVelocityHolonomic>>());
  // factory->registerType("EDGE_ACCELERATION",
  // std::make_shared<g2o::HyperGraphElementCreator<EdgeAcceleration>>());
  // factory->registerType("EDGE_ACCELERATION_START",
  // std::make_shared<g2o::HyperGraphElementCreator<EdgeAccelerationStart>>());
  // factory->registerType("EDGE_ACCELERATION_GOAL",
  // std::make_shared<g2o::HyperGraphElementCreator<EdgeAccelerationGoal>>());
  // factory->registerType("EDGE_ACCELERATION_HOLONOMIC",
  // std::make_shared<g2o::HyperGraphElementCreator<EdgeAccelerationHolonomic>>());
  // factory->registerType("EDGE_ACCELERATION_HOLONOMIC_START",
  // std::make_shared<g2o::HyperGraphElementCreator<EdgeAccelerationHolonomicStart>>());
  // factory->registerType("EDGE_ACCELERATION_HOLONOMIC_GOAL",
  // std::make_shared<g2o::HyperGraphElementCreator<EdgeAccelerationHolonomicGoal>>());
  // factory->registerType("EDGE_KINEMATICS_DIFF_DRIVE",
  // std::make_shared<g2o::HyperGraphElementCreator<EdgeKinematicsDiffDrive>>());
  // factory->registerType("EDGE_KINEMATICS_CARLIKE",
  // std::make_shared<g2o::HyperGraphElementCreator<EdgeKinematicsCarlike>>());
  // factory->registerType("EDGE_OBSTACLE",
  // std::make_shared<g2o::HyperGraphElementCreator<EdgeObstacle>>());
  // factory->registerType("EDGE_INFLATED_OBSTACLE",
  // std::make_shared<g2o::HyperGraphElementCreator<EdgeInflatedObstacle>>());
  // factory->registerType("EDGE_DYNAMIC_OBSTACLE",
  // std::make_shared<g2o::HyperGraphElementCreator<EdgeDynamicObstacle>>());
  // factory->registerType("EDGE_VIA_POINT",
  // std::make_shared<g2o::HyperGraphElementCreator<EdgeViaPoint>>());
  // factory->registerType("EDGE_PREFER_ROTDIR",
  // std::make_shared<g2o::HyperGraphElementCreator<EdgePreferRotDir>>());

  factory->registerType( "VERTEX_POSE", new g2o::HyperGraphElementCreator<VertexPose>());
  factory->registerType( "VERTEX_TIMEDIFF",new g2o::HyperGraphElementCreator<VertexTimeDiff>());

  factory->registerType("EDGE_TIME_OPTIMAL", new g2o::HyperGraphElementCreator<EdgeTimeOptimal>());
  factory->registerType("EDGE_SHORTEST_PATH", new g2o::HyperGraphElementCreator<EdgeShortestPath>());
  factory->registerType("EDGE_VELOCITY", new g2o::HyperGraphElementCreator<EdgeVelocity>());
  factory->registerType("EDGE_VELOCITY_HOLONOMIC", new g2o::HyperGraphElementCreator<EdgeVelocityHolonomic>());
  factory->registerType("EDGE_ACCELERATION", new g2o::HyperGraphElementCreator<EdgeAcceleration>());
  factory->registerType("EDGE_ACCELERATION_START",new  g2o::HyperGraphElementCreator<EdgeAccelerationStart>());
  factory->registerType("EDGE_ACCELERATION_GOAL", new g2o::HyperGraphElementCreator<EdgeAccelerationGoal>());
  factory->registerType("EDGE_ACCELERATION_HOLONOMIC", new g2o::HyperGraphElementCreator<EdgeAccelerationHolonomic>());
  factory->registerType("EDGE_ACCELERATION_HOLONOMIC_START", new g2o::HyperGraphElementCreator<EdgeAccelerationHolonomicStart>());
  factory->registerType("EDGE_ACCELERATION_HOLONOMIC_GOAL", new g2o::HyperGraphElementCreator<EdgeAccelerationHolonomicGoal>());
  factory->registerType("EDGE_KINEMATICS_DIFF_DRIVE", new g2o::HyperGraphElementCreator<EdgeKinematicsDiffDrive>());
  factory->registerType("EDGE_KINEMATICS_CARLIKE", new g2o::HyperGraphElementCreator<EdgeKinematicsCarlike>());
  factory->registerType("EDGE_OBSTACLE", new g2o::HyperGraphElementCreator<EdgeObstacle>());
  factory->registerType("EDGE_INFLATED_OBSTACLE", new g2o::HyperGraphElementCreator<EdgeInflatedObstacle>());
  factory->registerType("EDGE_DYNAMIC_OBSTACLE", new g2o::HyperGraphElementCreator<EdgeDynamicObstacle>());
  factory->registerType("EDGE_VIA_POINT", new g2o::HyperGraphElementCreator<EdgeViaPoint>());
  factory->registerType("EDGE_PREFER_ROTDIR", new g2o::HyperGraphElementCreator<EdgePreferRotDir>());
}

/*
 * initialize g2o optimizer Set solver settings here.
 * Return: pointer to new SparseOptimizer Object.
 */ // NOTE: gxt 配置g2o优化器
boost::shared_ptr<g2o::SparseOptimizer> TebOptimalPlanner::initOptimizer() {
  // Call register_g2o_types once, even for multiple TebOptimalPlanner instances
  // (thread-safe)
  static boost::once_flag flag = BOOST_ONCE_INIT;
  boost::call_once(&registerG2OTypes, flag);

  // allocating the optimizer
  boost::shared_ptr<g2o::SparseOptimizer> optimizer =
      boost::make_shared<g2o::SparseOptimizer>();
  std::unique_ptr<TEBLinearSolver> linear_solver( // 线性求解器，矩阵大小不确定（动态矩阵）
      new TEBLinearSolver());  // see typedef in optimization.h
  // std::unique_ptr<g2o::LinearSolverCSparse<
  //     g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1> >::PoseMatrixType> >
      // linear_solver(new TEBLinearSolver());
  linear_solver->setBlockOrdering(true);
  std::unique_ptr<TEBBlockSolver> block_solver(
      new TEBBlockSolver(std::move(linear_solver)));
  g2o::OptimizationAlgorithmLevenberg* solver =
      new g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));

  optimizer->setAlgorithm(solver);

  optimizer->initMultiThreading();  // required for >Eigen 3.1

  return optimizer;
}

bool TebOptimalPlanner::optimizeTEB(int iterations_innerloop,
                                    int iterations_outerloop,
                                    bool compute_cost_afterwards,
                                    double obst_cost_scale,
                                    double viapoint_cost_scale,
                                    bool alternative_time_cost) {
  if (cfg_->optim.optimization_activate == false) return false;

  bool success = false;
  optimized_ = false;

  double weight_multiplier = 1.0;

  // TODO(roesmann): we introduced the non-fast mode with the support of dynamic
  // obstacles
  //                (which leads to better results in terms of x-y-t homotopy
  //                planning).
  //                 however, we have not tested this mode intensively yet, so
  //                 we keep the legacy fast mode as default until we finish our
  //                 tests.
  bool fast_mode = !cfg_->obstacles.include_dynamic_obstacles;
  gDebugCol2(teb_.sizePoses());
  gDebugCol2(teb_.sizeTimeDiffs());
  for (int i = 0; i < iterations_outerloop; ++i) {
    if (cfg_->trajectory.teb_autosize) {
      // teb_.autoResize(cfg_->trajectory.dt_ref,
      // cfg_->trajectory.dt_hysteresis, cfg_->trajectory.min_samples,
      // cfg_->trajectory.max_samples);
      teb_.autoResize(cfg_->trajectory.dt_ref, cfg_->trajectory.dt_hysteresis,
                      cfg_->trajectory.min_samples,
                      cfg_->trajectory.max_samples, fast_mode);
    }

    success = buildGraph(weight_multiplier);
    if (!success) {
      clearGraph();
      return false;
    }
    success = optimizeGraph(iterations_innerloop, false);
    if (!success) {
      clearGraph();
      return false;
    }
    optimized_ = true;

    if (compute_cost_afterwards &&
        i == iterations_outerloop -
                 1)  // compute cost vec only in the last iteration
      computeCurrentCost(obst_cost_scale, viapoint_cost_scale,
                         alternative_time_cost);

    clearGraph();

    weight_multiplier *= cfg_->optim.weight_adapt_factor;
  }

  return true;
}

bool TebOptimalPlanner::optimizeTEBCeres(int iterations_innerloop,
                                    int iterations_outerloop,
                                    bool compute_cost_afterwards,
                                    double obst_cost_scale,
                                    double viapoint_cost_scale,
                                    bool alternative_time_cost) {
  if (cfg_->optim.optimization_activate == false) return false;

  bool success = false;
  optimized_ = false;

  double weight_multiplier = 1.0;

  // TODO(roesmann): we introduced the non-fast mode with the support of dynamic
  // obstacles
  //                (which leads to better results in terms of x-y-t homotopy
  //                planning).
  //                 however, we have not tested this mode intensively yet, so
  //                 we keep the legacy fast mode as default until we finish our
  //                 tests.
  bool fast_mode = !cfg_->obstacles.include_dynamic_obstacles;
  gDebugCol2(teb_.sizePoses());
  gDebugCol2(teb_.sizeTimeDiffs());
  for (int i = 0; i < 1; ++i) {
    if (cfg_->trajectory.teb_autosize) {
      // teb_.autoResize(cfg_->trajectory.dt_ref,
      // cfg_->trajectory.dt_hysteresis, cfg_->trajectory.min_samples,
      // cfg_->trajectory.max_samples);
      teb_.autoResize(cfg_->trajectory.dt_ref, cfg_->trajectory.dt_hysteresis,
                      cfg_->trajectory.min_samples,
                      cfg_->trajectory.max_samples, fast_mode);
    }

    success = buildGraphCeres(weight_multiplier);
    // success = buildGraph(weight_multiplier);
    if (!success) {
      // clearGraph();
      clearGraphCeres();
      return false;
    }


    // success = optimizeGraph(iterations_innerloop, false);
    success = optimizeGraphCeres(iterations_innerloop, false);
    if (!success) {
      // clearGraph();
      clearGraphCeres();
      return false;
    }
    optimized_ = true;

    if (compute_cost_afterwards &&
        i == iterations_outerloop -
                 1)  // compute cost vec only in the last iteration
      computeCurrentCost(obst_cost_scale, viapoint_cost_scale,
                         alternative_time_cost);

    // clearGraph();
    clearGraphCeres();

    weight_multiplier *= cfg_->optim.weight_adapt_factor;
  }

  // std::cout << summary.FullReport() << '\n';
  // gDebugError("std::terminate");
  return true;
}



void TebOptimalPlanner::setVelocityStart(const Twist& vel_start) {
  vel_start_.first = true;
  vel_start_.second.linear.x() = vel_start.linear.x();
  vel_start_.second.linear.y() = vel_start.linear.y();
  vel_start_.second.angular.z() = vel_start.angular.z();
}

void TebOptimalPlanner::setVelocityGoal(const Twist& vel_goal) {
  vel_goal_.first = true;
  vel_goal_.second = vel_goal;
}

bool TebOptimalPlanner::plan(const std::vector<PoseStamped>& initial_plan,
                             const Twist* start_vel, bool free_goal_vel) {
  if (!teb_.isInit()) {
    teb_.initTrajectoryToGoal(
        initial_plan, cfg_->robot.max_vel_x, cfg_->robot.max_vel_theta,
        cfg_->trajectory.global_plan_overwrite_orientation,
        cfg_->trajectory.min_samples,
        cfg_->trajectory.allow_init_with_backwards_motion);
  } else  // warm start
  {
    PoseSE2 start_(initial_plan.front().pose);
    PoseSE2 goal_(initial_plan.back().pose);
    if (teb_.sizePoses() > 0 &&
        (goal_.position() - teb_.BackPose().position()).norm() <
            cfg_->trajectory.force_reinit_new_goal_dist &&
        fabs(g2o::normalize_theta(goal_.theta() - teb_.BackPose().theta())) <
            cfg_->trajectory
                .force_reinit_new_goal_angular)  // actual warm start!
      teb_.updateAndPruneTEB(start_, goal_,
                             cfg_->trajectory.min_samples);  // update TEB
    else  // goal too far away -> reinit
    {
      printf(
          "New goal: distance to existing goal is higher than the specified "
          "threshold. Reinitalizing trajectories.");
      teb_.clearTimedElasticBand();
      teb_.initTrajectoryToGoal(
          initial_plan, cfg_->robot.max_vel_x, cfg_->robot.max_vel_theta,
          cfg_->trajectory.global_plan_overwrite_orientation,
          cfg_->trajectory.min_samples,
          cfg_->trajectory.allow_init_with_backwards_motion);
    }
  }
  if (start_vel) setVelocityStart(*start_vel);
  if (free_goal_vel)
    setVelocityGoalFree();
  else
    vel_goal_.first =
        true;  // we just reactivate and use the previously set velocity (should
               // be zero if nothing was modified)

  // now optimize
  return optimizeTEB(cfg_->optim.no_inner_iterations,
                     cfg_->optim.no_outer_iterations);
}

bool TebOptimalPlanner::plan(const Pose& start, const Pose& goal,
                             const Twist* start_vel, bool free_goal_vel) {
  PoseSE2 start_(start);
  PoseSE2 goal_(goal);
  return plan(start_, goal_, start_vel);
}

bool TebOptimalPlanner::plan(const PoseSE2& start, const PoseSE2& goal,
                             const Twist* start_vel, bool free_goal_vel) {
  if (!teb_.isInit()) {
    // init trajectory
    teb_.initTrajectoryToGoal(
        start, goal, 0, cfg_->robot.max_vel_x, cfg_->trajectory.min_samples,
        cfg_->trajectory
            .allow_init_with_backwards_motion);  // 0 intermediate samples, but
                                                 // dt=1 -> autoResize will add
                                                 // more samples before calling
                                                 // first optimization
  } else                                         // warm start
  {
    if (teb_.sizePoses() > 0 &&
        (goal.position() - teb_.BackPose().position()).norm() <
            cfg_->trajectory.force_reinit_new_goal_dist &&
        fabs(g2o::normalize_theta(goal.theta() - teb_.BackPose().theta())) <
            cfg_->trajectory
                .force_reinit_new_goal_angular)  // actual warm start!
      teb_.updateAndPruneTEB(start, goal, cfg_->trajectory.min_samples);
    else  // goal too far away -> reinit
    {
      // ROS_DEBUG("New goal: distance to existing goal is higher than the
      // specified threshold. Reinitalizing trajectories.");
      teb_.clearTimedElasticBand();
      teb_.initTrajectoryToGoal(
          start, goal, 0, cfg_->robot.max_vel_x, cfg_->trajectory.min_samples,
          cfg_->trajectory.allow_init_with_backwards_motion);
    }
  }
  if (start_vel) setVelocityStart(*start_vel);
  if (free_goal_vel)
    setVelocityGoalFree();
  else
    vel_goal_.first =
        true;  // we just reactivate and use the previously set velocity (should
               // be zero if nothing was modified)

  // now optimize
  if(config.gxt.g2o_or_ceres=="ceres") {
    return optimizeTEBCeres(cfg_->optim.no_inner_iterations,cfg_->optim.no_outer_iterations);
  }
  return optimizeTEB(cfg_->optim.no_inner_iterations,
                     cfg_->optim.no_outer_iterations);
}


bool TebOptimalPlanner::buildGraphCeres(double weight_multiplier) {
  // if (!optimizer_->edges().empty() || !optimizer_->vertices().empty()) {
  //   // ROS_WARN("Cannot build graph, because it is not empty. Call
  //   // graphClear()!");
  //   return false;
  // }

  // add TEB vertices
  AddTEBVerticesCeres();





  // 先不管障碍物 TODO: 障碍物要加
  // add Edges (local cost functions)
  // if (cfg_->obstacles.legacy_obstacle_association)
  //   AddEdgesObstaclesLegacy(weight_multiplier);
  // else
  //   AddEdgesObstacles(weight_multiplier);
//  AddEdgesObstacles(weight_multiplier);
  AddEdgesObstaclesCeres(weight_multiplier);

  // if (cfg_->obstacles.include_dynamic_obstacles) AddEdgesDynamicObstacles();

  // 经过点也先不加
  // AddEdgesViaPoints();

  // 速度要加
  AddEdgesVelocityCeres();

  // 加速度要加 //
  AddEdgesAccelerationCeres();

  // 时间最优要加
  AddEdgesTimeOptimalCeres();

  // 最短路径要加
  AddEdgesShortestPathCeres();

  // if (cfg_->robot.min_turning_radius == 0 ||
  //     cfg_->optim.weight_kinematics_turning_radius == 0)
  //   AddEdgesKinematicsDiffDrive();  // we have a differential drive robot
  // else
  //   AddEdgesKinematicsCarlike();  // we have a carlike robot since the turning
  //                                 // radius is bounded from below.
  //
  // 因为最小转弯半径为0,所以直接运行这个
  AddEdgesKinematicsDiffDriveCeres(); 



  // 固定第一个位姿点和最后一个位姿点
  problem.SetParameterBlockConstant(&teb_.PoseVertex(0)->x());
  problem.SetParameterBlockConstant(&teb_.PoseVertex(0)->y());
  problem.SetParameterBlockConstant(&teb_.PoseVertex(0)->theta());
  problem.SetParameterBlockConstant(&teb_.PoseVertex(teb_.sizePoses()-1)->x());
  problem.SetParameterBlockConstant(&teb_.PoseVertex(teb_.sizePoses()-1)->y());
  problem.SetParameterBlockConstant(&teb_.PoseVertex(teb_.sizePoses()-1)->theta());


  // 这里啥也没做
  // AddEdgesPreferRotDir();

  return true;
}

bool TebOptimalPlanner::buildGraph(double weight_multiplier) {
  if (!optimizer_->edges().empty() || !optimizer_->vertices().empty()) {
    // ROS_WARN("Cannot build graph, because it is not empty. Call
    // graphClear()!");
    return false;
  }

  // add TEB vertices
  AddTEBVertices();

  // add Edges (local cost functions)
  if (cfg_->obstacles.legacy_obstacle_association)
    AddEdgesObstaclesLegacy(weight_multiplier);
  else
    AddEdgesObstacles(weight_multiplier);

  if (cfg_->obstacles.include_dynamic_obstacles) AddEdgesDynamicObstacles();

  AddEdgesViaPoints();

  AddEdgesVelocity();

  AddEdgesAcceleration();

  AddEdgesTimeOptimal();

  AddEdgesShortestPath();

  if (cfg_->robot.min_turning_radius == 0 ||
      cfg_->optim.weight_kinematics_turning_radius == 0)
    AddEdgesKinematicsDiffDrive();  // we have a differential drive robot
  else
    AddEdgesKinematicsCarlike();  // we have a carlike robot since the turning
                                  // radius is bounded from below.

  AddEdgesPreferRotDir();

  return true;
}

bool TebOptimalPlanner::optimizeGraph(int no_iterations, bool clear_after) {
  if (cfg_->robot.max_vel_x < 0.01) {
    // ROS_WARN("optimizeGraph(): Robot Max Velocity is smaller than 0.01m/s.
    // Optimizing aborted...");
    if (clear_after) clearGraph();
    return false;
  }

  if (!teb_.isInit() || teb_.sizePoses() < cfg_->trajectory.min_samples) {
    // ROS_WARN("optimizeGraph(): TEB is empty or has too less elements.
    // Skipping optimization.");
    if (clear_after) clearGraph();
    return false;
  }

  optimizer_->setVerbose(cfg_->optim.optimization_verbose);
  optimizer_->initializeOptimization();

  int iter = optimizer_->optimize(no_iterations);

  // Save Hessian for visualization
  //  g2o::OptimizationAlgorithmLevenberg* lm =
  //  dynamic_cast<g2o::OptimizationAlgorithmLevenberg*> (optimizer_->solver());
  //  lm->solver()->saveHessian("~/MasterThesis/Matlab/Hessian.txt");

  if (!iter) {
    // ROS_ERROR("optimizeGraph(): Optimization failed! iter=%i", iter);
    return false;
  }

  if (clear_after) clearGraph();

  return true;
}


bool TebOptimalPlanner::optimizeGraphCeres(int no_iterations, bool clear_after) {
  if (cfg_->robot.max_vel_x < 0.01) {
    if (clear_after) clearGraphCeres();
    return false;
  }

  if (!teb_.isInit() || teb_.sizePoses() < cfg_->trajectory.min_samples) {
    // ROS_WARN("optimizeGraph(): TEB is empty or has too less elements.
    // Skipping optimization.");
    if (clear_after) clearGraphCeres();
    return false;
  }

  ceres::Solver::Options options;
  // options.num_threads=1;
  options.max_num_iterations = 50;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  gDebugCol4("=========");
  gDebugCol4(&problem);
  // optimizer_->setVerbose(cfg_->optim.optimization_verbose);
  // optimizer_->initializeOptimization();

  // int iter = optimizer_->optimize(no_iterations);
  bool iter=summary.IsSolutionUsable();

  // Save Hessian for visualization
  //  g2o::OptimizationAlgorithmLevenberg* lm =
  //  dynamic_cast<g2o::OptimizationAlgorithmLevenberg*> (optimizer_->solver());
  //  lm->solver()->saveHessian("~/MasterThesis/Matlab/Hessian.txt");

  if (!iter) {
    gDebugCol3("cannot use");
    // ROS_ERROR("optimizeGraph(): Optimization failed! iter=%i", iter);
    return false;
  }

  if (clear_after) clearGraphCeres();

  return true;
}

void TebOptimalPlanner::clearGraph() {
  // clear optimizer states
  if (optimizer_) {
    // optimizer.edges().clear(); // optimizer.clear deletes edges!!! Therefore
    // do not run optimizer.edges().clear()
    optimizer_->vertices()
        .clear();  // neccessary, because optimizer->clear deletes
                   // pointer-targets (therefore it deletes TEB states!)
    optimizer_->clear();
  }
}


void TebOptimalPlanner::clearGraphCeres() {
  if (optimizer_) {
    ceres::Problem problem2;
    problem=std::move(problem2);
  }
}

void TebOptimalPlanner::AddTEBVertices() {
  // add vertices to graph
  // ROS_DEBUG_COND(cfg_->optim.optimization_verbose, "Adding TEB vertices
  // ...");
  unsigned int id_counter = 0;  // used for vertices ids
  for (int i = 0; i < teb_.sizePoses(); ++i) {
    teb_.PoseVertex(i)->setId(id_counter++);
    optimizer_->addVertex(teb_.PoseVertex(i));
    if (teb_.sizeTimeDiffs() != 0 && i < teb_.sizeTimeDiffs()) {
      teb_.TimeDiffVertex(i)->setId(id_counter++);
      optimizer_->addVertex(teb_.TimeDiffVertex(i));
    }
  }
}

// ceres 不用像g2o先添加顶点后添加边，而是直接添加约束
void TebOptimalPlanner::AddTEBVerticesCeres() {
  // unsigned int id_counter = 0;  // used for vertices ids
  // for (int i = 0; i < teb_.sizePoses(); ++i) {
  //   teb_.PoseVertex(i)->setId(id_counter++);
  //   optimizer_->addVertex(teb_.PoseVertex(i));
  //   if (teb_.sizeTimeDiffs() != 0 && i < teb_.sizeTimeDiffs()) {
  //     teb_.TimeDiffVertex(i)->setId(id_counter++);
  //     optimizer_->addVertex(teb_.TimeDiffVertex(i));
  //   }
  // }
}

void TebOptimalPlanner::AddEdgesObstacles(double weight_multiplier) {
  if (cfg_->optim.weight_obstacle == 0 || weight_multiplier == 0 ||
      obstacles_ == nullptr)
    return;  // if weight equals zero skip adding edges!

  bool inflated =
      cfg_->obstacles.inflation_dist > cfg_->obstacles.min_obstacle_dist;

  Eigen::Matrix<double, 1, 1> information;
  information.fill(cfg_->optim.weight_obstacle * weight_multiplier);

  Eigen::Matrix<double, 2, 2> information_inflated;
  information_inflated(0, 0) = cfg_->optim.weight_obstacle * weight_multiplier;
  information_inflated(1, 1) = cfg_->optim.weight_inflation;
  information_inflated(0, 1) = information_inflated(1, 0) = 0;

  std::vector<Obstacle*> relevant_obstacles;
  relevant_obstacles.reserve(obstacles_->size());

  // iterate all teb points (skip first and last)
  for (int i = 1; i < teb_.sizePoses() - 1; ++i) {
    double left_min_dist = std::numeric_limits<double>::max();
    double right_min_dist = std::numeric_limits<double>::max();
    Obstacle* left_obstacle = nullptr;
    Obstacle* right_obstacle = nullptr;

    relevant_obstacles.clear();

    const Eigen::Vector2d pose_orient = teb_.Pose(i).orientationUnitVec();

    // iterate obstacles
    for (const ObstaclePtr& obst : *obstacles_) {
      // we handle dynamic obstacles differently below
      if (cfg_->obstacles.include_dynamic_obstacles && obst->isDynamic())
        continue;

      // calculate distance to robot model
      double dist = robot_model_->calculateDistance(teb_.Pose(i), obst.get());

      // force considering obstacle if really close to the current pose
      if (dist <
          cfg_->obstacles.min_obstacle_dist *
              cfg_->obstacles.obstacle_association_force_inclusion_factor) {
        relevant_obstacles.push_back(obst.get());
        continue;
      }
      // cut-off distance
      if (dist > cfg_->obstacles.min_obstacle_dist *
                     cfg_->obstacles.obstacle_association_cutoff_factor)
        continue;

      // determine side (left or right) and assign obstacle if closer than the
      // previous one
      if (cross2d(pose_orient, obst->getCentroid()) > 0)  // left
      {
        if (dist < left_min_dist) {
          left_min_dist = dist;
          left_obstacle = obst.get();
        }
      } else {
        if (dist < right_min_dist) {
          right_min_dist = dist;
          right_obstacle = obst.get();
        }
      }
    }

    // create obstacle edges
    if (left_obstacle) {
      if (inflated) {
        EdgeInflatedObstacle* dist_bandpt_obst = new EdgeInflatedObstacle;
        dist_bandpt_obst->setVertex(0, teb_.PoseVertex(i));
        dist_bandpt_obst->setInformation(information_inflated);
        dist_bandpt_obst->setParameters(*cfg_, robot_model_.get(),
                                        left_obstacle);
        optimizer_->addEdge(dist_bandpt_obst);
      } else {
        EdgeObstacle* dist_bandpt_obst = new EdgeObstacle;
        dist_bandpt_obst->setVertex(0, teb_.PoseVertex(i));
        dist_bandpt_obst->setInformation(information);
        dist_bandpt_obst->setParameters(*cfg_, robot_model_.get(),
                                        left_obstacle);
        optimizer_->addEdge(dist_bandpt_obst);
      }
    }

    if (right_obstacle) {
      if (inflated) {
        EdgeInflatedObstacle* dist_bandpt_obst = new EdgeInflatedObstacle;
        dist_bandpt_obst->setVertex(0, teb_.PoseVertex(i));
        dist_bandpt_obst->setInformation(information_inflated);
        dist_bandpt_obst->setParameters(*cfg_, robot_model_.get(),
                                        right_obstacle);
        optimizer_->addEdge(dist_bandpt_obst);
      } else {
        EdgeObstacle* dist_bandpt_obst = new EdgeObstacle;
        dist_bandpt_obst->setVertex(0, teb_.PoseVertex(i));
        dist_bandpt_obst->setInformation(information);
        dist_bandpt_obst->setParameters(*cfg_, robot_model_.get(),
                                        right_obstacle);
        optimizer_->addEdge(dist_bandpt_obst);
      }
    }

    for (const Obstacle* obst : relevant_obstacles) {
      if (inflated) {
        EdgeInflatedObstacle* dist_bandpt_obst = new EdgeInflatedObstacle;
        dist_bandpt_obst->setVertex(0, teb_.PoseVertex(i));
        dist_bandpt_obst->setInformation(information_inflated);
        dist_bandpt_obst->setParameters(*cfg_, robot_model_.get(), obst);
        optimizer_->addEdge(dist_bandpt_obst);
      } else {
        EdgeObstacle* dist_bandpt_obst = new EdgeObstacle;
        dist_bandpt_obst->setVertex(0, teb_.PoseVertex(i));
        dist_bandpt_obst->setInformation(information);
        dist_bandpt_obst->setParameters(*cfg_, robot_model_.get(), obst);
        optimizer_->addEdge(dist_bandpt_obst);
      }
    }
  }
}


void TebOptimalPlanner::AddEdgesObstaclesCeres(double weight_multiplier) {
  if (cfg_->optim.weight_obstacle == 0 || weight_multiplier == 0 ||
      obstacles_ == nullptr)
    return;  // if weight equals zero skip adding edges!

  bool inflated =
      cfg_->obstacles.inflation_dist > cfg_->obstacles.min_obstacle_dist;

  Eigen::Matrix<double, 1, 1> information;
  information.fill(cfg_->optim.weight_obstacle * weight_multiplier);

  Eigen::Matrix<double, 2, 2> information_inflated;
  information_inflated(0, 0) = cfg_->optim.weight_obstacle * weight_multiplier;
  information_inflated(1, 1) = cfg_->optim.weight_inflation;
  information_inflated(0, 1) = information_inflated(1, 0) = 0;

  const Eigen::Matrix<double,1,1> sqrt_information= information.llt().matrixL();
  const Eigen::Matrix2d sqrt_information_inflated= information_inflated.llt().matrixL();

  std::vector<Obstacle*> relevant_obstacles;
  relevant_obstacles.reserve(obstacles_->size());

  // iterate all teb points (skip first and last)
  for (int i = 1; i < teb_.sizePoses() - 1; ++i) {
    double left_min_dist = std::numeric_limits<double>::max();
    double right_min_dist = std::numeric_limits<double>::max();
    Obstacle* left_obstacle = nullptr;
    Obstacle* right_obstacle = nullptr;

    relevant_obstacles.clear();

    const Eigen::Vector2d pose_orient = teb_.Pose(i).orientationUnitVec();

    // iterate obstacles
    for (const ObstaclePtr& obst : *obstacles_) {
      // we handle dynamic obstacles differently below
      if (cfg_->obstacles.include_dynamic_obstacles && obst->isDynamic())
        continue;

      // calculate distance to robot model
      double dist = robot_model_->calculateDistance(teb_.Pose(i), obst.get());

      // force considering obstacle if really close to the current pose
      if (dist <
          cfg_->obstacles.min_obstacle_dist *
              cfg_->obstacles.obstacle_association_force_inclusion_factor) {
        relevant_obstacles.push_back(obst.get());
        continue;
      }
      // cut-off distance
      if (dist > cfg_->obstacles.min_obstacle_dist *
                     cfg_->obstacles.obstacle_association_cutoff_factor)
        continue;

      // determine side (left or right) and assign obstacle if closer than the
      // previous one
      if (cross2d(pose_orient, obst->getCentroid()) > 0)  // left
      {
        if (dist < left_min_dist) {
          left_min_dist = dist;
          left_obstacle = obst.get();
        }
      } else {
        if (dist < right_min_dist) {
          right_min_dist = dist;
          right_obstacle = obst.get();
        }
      }
    }


    // create obstacle edges
    if (left_obstacle) {
      if (inflated) {
        // EdgeInflatedObstacle* dist_bandpt_obst = new EdgeInflatedObstacle;
        // dist_bandpt_obst->setVertex(0, teb_.PoseVertex(i));
        // dist_bandpt_obst->setInformation(information_inflated);
        // dist_bandpt_obst->setParameters(*cfg_, robot_model_.get(),
        //                                 left_obstacle);
        // optimizer_->addEdge(dist_bandpt_obst);
        //
        // ceres::CostFunction* cost_function =
        //     new ceres::AutoDiffCostFunction<ceres::CeresTebInflatedObstacle, 2,
        //                                     1, 1>(
        //         new ceres::CeresTebInflatedObstacle(
        //             sqrt_information_inflated, robot_model_.get(), left_obstacle));

        ceres::CostFunction* cost_function = new ceres::NumericDiffCostFunction<
            ceres::CeresTebInflatedObstacleNumeric, ceres::CENTRAL, 2, 1, 1>(
            new ceres::CeresTebInflatedObstacleNumeric(
                sqrt_information_inflated, robot_model_.get(), left_obstacle));

        problem.AddResidualBlock(cost_function, nullptr,
                                 &teb_.PoseVertex(i)->x(),
                                 &teb_.PoseVertex(i)->y());
      } else {
        gDebugError("ERROR")<<FILE_LINE;
        EdgeObstacle* dist_bandpt_obst = new EdgeObstacle;
        dist_bandpt_obst->setVertex(0, teb_.PoseVertex(i));
        dist_bandpt_obst->setInformation(information);
        dist_bandpt_obst->setParameters(*cfg_, robot_model_.get(),
                                        left_obstacle);
        optimizer_->addEdge(dist_bandpt_obst);
      }
    }

    if (right_obstacle) {
      if (inflated) {
        // EdgeInflatedObstacle* dist_bandpt_obst = new EdgeInflatedObstacle;
        // dist_bandpt_obst->setVertex(0, teb_.PoseVertex(i));
        // dist_bandpt_obst->setInformation(information_inflated);
        // dist_bandpt_obst->setParameters(*cfg_, robot_model_.get(),
        //                                 right_obstacle);
        // optimizer_->addEdge(dist_bandpt_obst);

        // ceres::CostFunction* cost_function =
        //     new ceres::AutoDiffCostFunction<ceres::CeresTebInflatedObstacle, 2,
        //                                     1, 1>(
        //         new ceres::CeresTebInflatedObstacle(
        //             sqrt_information_inflated, robot_model_.get(), right_obstacle));

        ceres::CostFunction* cost_function =
            new ceres::NumericDiffCostFunction<ceres::CeresTebInflatedObstacleNumeric,ceres::CENTRAL, 2,
                                            1, 1>(
                new ceres::CeresTebInflatedObstacleNumeric(
                    sqrt_information_inflated, robot_model_.get(), right_obstacle));

        problem.AddResidualBlock(cost_function, nullptr,
                                 &teb_.PoseVertex(i)->x(),
                                 &teb_.PoseVertex(i)->y());
      } else {
        gDebugError("ERROR")<<FILE_LINE;
        EdgeObstacle* dist_bandpt_obst = new EdgeObstacle;
        dist_bandpt_obst->setVertex(0, teb_.PoseVertex(i));
        dist_bandpt_obst->setInformation(information);
        dist_bandpt_obst->setParameters(*cfg_, robot_model_.get(),
                                        right_obstacle);
        optimizer_->addEdge(dist_bandpt_obst);
      }
    }

    for (const Obstacle* obst : relevant_obstacles) {
      if (inflated) {
        // EdgeInflatedObstacle* dist_bandpt_obst = new EdgeInflatedObstacle;
        // dist_bandpt_obst->setVertex(0, teb_.PoseVertex(i));
        // dist_bandpt_obst->setInformation(information_inflated);
        // dist_bandpt_obst->setParameters(*cfg_, robot_model_.get(), obst);
        // optimizer_->addEdge(dist_bandpt_obst);

        // TODO debug point
        // PoseSE2 temp;
        // robot_model_->calculateDistance(temp, obst);

        // ceres::CostFunction* cost_function =
        //     new ceres::AutoDiffCostFunction<ceres::CeresTebInflatedObstacle, 2,
        //                                     1, 1>(
        //         new ceres::CeresTebInflatedObstacle(
        //             sqrt_information_inflated, robot_model_.get(), obst));

        ceres::CostFunction* cost_function =
            new ceres::NumericDiffCostFunction<ceres::CeresTebInflatedObstacleNumeric,ceres::CENTRAL, 2,
                                            1, 1>(
                new ceres::CeresTebInflatedObstacleNumeric(
                    sqrt_information_inflated, robot_model_.get(), obst));

        problem.AddResidualBlock(cost_function, nullptr,
                                 &teb_.PoseVertex(i)->x(),
                                 &teb_.PoseVertex(i)->y());
      } else {
        gDebugError("ERROR")<<FILE_LINE;
        EdgeObstacle* dist_bandpt_obst = new EdgeObstacle;
        dist_bandpt_obst->setVertex(0, teb_.PoseVertex(i));
        dist_bandpt_obst->setInformation(information);
        dist_bandpt_obst->setParameters(*cfg_, robot_model_.get(), obst);
        optimizer_->addEdge(dist_bandpt_obst);
      }
    }
  }
}

void TebOptimalPlanner::AddEdgesObstaclesLegacy(double weight_multiplier) {
  if (cfg_->optim.weight_obstacle == 0 || weight_multiplier == 0 ||
      obstacles_ == nullptr)
    return;  // if weight equals zero skip adding edges!

  Eigen::Matrix<double, 1, 1> information;
  information.fill(cfg_->optim.weight_obstacle * weight_multiplier);

  Eigen::Matrix<double, 2, 2> information_inflated;
  information_inflated(0, 0) = cfg_->optim.weight_obstacle * weight_multiplier;
  information_inflated(1, 1) = cfg_->optim.weight_inflation;
  information_inflated(0, 1) = information_inflated(1, 0) = 0;

  bool inflated =
      cfg_->obstacles.inflation_dist > cfg_->obstacles.min_obstacle_dist;

  for (ObstContainer::const_iterator obst = obstacles_->begin();
       obst != obstacles_->end(); ++obst) {
    if (cfg_->obstacles.include_dynamic_obstacles &&
        (*obst)->isDynamic())  // we handle dynamic obstacles differently below
      continue;

    int index;

    if (cfg_->obstacles.obstacle_poses_affected >= teb_.sizePoses())
      index = teb_.sizePoses() / 2;
    else
      index = teb_.findClosestTrajectoryPose(*(obst->get()));

    // check if obstacle is outside index-range between start and goal
    if ((index <= 1) ||
        (index > teb_.sizePoses() -
                     2))  // start and goal are fixed and findNearestBandpoint
                          // finds first or last conf if intersection point is
                          // outside the range
      continue;

    if (inflated) {
      EdgeInflatedObstacle* dist_bandpt_obst = new EdgeInflatedObstacle;
      dist_bandpt_obst->setVertex(0, teb_.PoseVertex(index));
      dist_bandpt_obst->setInformation(information_inflated);
      dist_bandpt_obst->setParameters(*cfg_, robot_model_.get(), obst->get());
      optimizer_->addEdge(dist_bandpt_obst);
    } else {
      EdgeObstacle* dist_bandpt_obst = new EdgeObstacle;
      dist_bandpt_obst->setVertex(0, teb_.PoseVertex(index));
      dist_bandpt_obst->setInformation(information);
      dist_bandpt_obst->setParameters(*cfg_, robot_model_.get(), obst->get());
      optimizer_->addEdge(dist_bandpt_obst);
    }

    for (int neighbourIdx = 0;
         neighbourIdx < floor(cfg_->obstacles.obstacle_poses_affected / 2);
         neighbourIdx++) {
      if (index + neighbourIdx < teb_.sizePoses()) {
        if (inflated) {
          EdgeInflatedObstacle* dist_bandpt_obst_n_r = new EdgeInflatedObstacle;
          dist_bandpt_obst_n_r->setVertex(
              0, teb_.PoseVertex(index + neighbourIdx));
          dist_bandpt_obst_n_r->setInformation(information_inflated);
          dist_bandpt_obst_n_r->setParameters(*cfg_, robot_model_.get(),
                                              obst->get());
          optimizer_->addEdge(dist_bandpt_obst_n_r);
        } else {
          EdgeObstacle* dist_bandpt_obst_n_r = new EdgeObstacle;
          dist_bandpt_obst_n_r->setVertex(
              0, teb_.PoseVertex(index + neighbourIdx));
          dist_bandpt_obst_n_r->setInformation(information);
          dist_bandpt_obst_n_r->setParameters(*cfg_, robot_model_.get(),
                                              obst->get());
          optimizer_->addEdge(dist_bandpt_obst_n_r);
        }
      }
      if (index - neighbourIdx >=
          0)  // needs to be casted to int to allow negative values
      {
        if (inflated) {
          EdgeInflatedObstacle* dist_bandpt_obst_n_l = new EdgeInflatedObstacle;
          dist_bandpt_obst_n_l->setVertex(
              0, teb_.PoseVertex(index - neighbourIdx));
          dist_bandpt_obst_n_l->setInformation(information_inflated);
          dist_bandpt_obst_n_l->setParameters(*cfg_, robot_model_.get(),
                                              obst->get());
          optimizer_->addEdge(dist_bandpt_obst_n_l);
        } else {
          EdgeObstacle* dist_bandpt_obst_n_l = new EdgeObstacle;
          dist_bandpt_obst_n_l->setVertex(
              0, teb_.PoseVertex(index - neighbourIdx));
          dist_bandpt_obst_n_l->setInformation(information);
          dist_bandpt_obst_n_l->setParameters(*cfg_, robot_model_.get(),
                                              obst->get());
          optimizer_->addEdge(dist_bandpt_obst_n_l);
        }
      }
    }
  }
}

void TebOptimalPlanner::AddEdgesDynamicObstacles(double weight_multiplier) {
  if (cfg_->optim.weight_obstacle == 0 || weight_multiplier == 0 ||
      obstacles_ == NULL)
    return;  // if weight equals zero skip adding edges!

  Eigen::Matrix<double, 2, 2> information;
  information(0, 0) = cfg_->optim.weight_dynamic_obstacle * weight_multiplier;
  information(1, 1) = cfg_->optim.weight_dynamic_obstacle_inflation;
  information(0, 1) = information(1, 0) = 0;

  for (ObstContainer::const_iterator obst = obstacles_->begin();
       obst != obstacles_->end(); ++obst) {
    if (!(*obst)->isDynamic()) continue;

    // Skip first and last pose, as they are fixed
    double time = teb_.TimeDiff(0);
    for (int i = 1; i < teb_.sizePoses() - 1; ++i) {
      EdgeDynamicObstacle* dynobst_edge = new EdgeDynamicObstacle(time);
      dynobst_edge->setVertex(0, teb_.PoseVertex(i));
      dynobst_edge->setInformation(information);
      dynobst_edge->setParameters(*cfg_, robot_model_.get(), obst->get());
      optimizer_->addEdge(dynobst_edge);
      time +=
          teb_.TimeDiff(i);  // we do not need to check the time diff bounds,
                             // since we iterate to "< sizePoses()-1".
    }
  }
}

void TebOptimalPlanner::AddEdgesViaPoints() {
  if (cfg_->optim.weight_viapoint == 0 || via_points_ == NULL ||
      via_points_->empty())
    return;  // if weight equals zero skip adding edges!

  int start_pose_idx = 0;

  int n = teb_.sizePoses();
  if (n < 3)  // we do not have any degrees of freedom for reaching via-points
    return;

  for (ViaPointContainer::const_iterator vp_it = via_points_->begin();
       vp_it != via_points_->end(); ++vp_it) {
    int index = teb_.findClosestTrajectoryPose(*vp_it, NULL, start_pose_idx);
    if (cfg_->trajectory.via_points_ordered)
      start_pose_idx =
          index +
          2;  // skip a point to have a DOF inbetween for further via-points

    // check if point conicides with goal or is located behind it
    if (index > n - 2)
      index =
          n - 2;  // set to a pose before the goal, since we can move it away!
    // check if point coincides with start or is located before it
    if (index < 1) {
      if (cfg_->trajectory.via_points_ordered) {
        index = 1;  // try to connect the via point with the second (and
                    // non-fixed) pose. It is likely that autoresize adds new
                    // poses inbetween later.
      } else {
        // ROS_DEBUG("TebOptimalPlanner::AddEdgesViaPoints(): skipping a
        // via-point that is close or behind the current robot pose.");
        continue;  // skip via points really close or behind the current robot
                   // pose
      }
    }
    Eigen::Matrix<double, 1, 1> information;
    information.fill(cfg_->optim.weight_viapoint);

    EdgeViaPoint* edge_viapoint = new EdgeViaPoint;
    edge_viapoint->setVertex(0, teb_.PoseVertex(index));
    edge_viapoint->setInformation(information);
    edge_viapoint->setParameters(*cfg_, &(*vp_it));
    optimizer_->addEdge(edge_viapoint);
  }
}

void TebOptimalPlanner::AddEdgesVelocity() {
  if (cfg_->robot.max_vel_y == 0)  // non-holonomic robot
  {
    if (cfg_->optim.weight_max_vel_x == 0 &&
        cfg_->optim.weight_max_vel_theta == 0)
      return;  // if weight equals zero skip adding edges!

    int n = teb_.sizePoses();
    Eigen::Matrix<double, 2, 2> information;
    information(0, 0) = cfg_->optim.weight_max_vel_x;
    information(1, 1) = cfg_->optim.weight_max_vel_theta;
    information(0, 1) = 0.0;
    information(1, 0) = 0.0;

    for (int i = 0; i < n - 1; ++i) {
      EdgeVelocity* velocity_edge = new EdgeVelocity;
      velocity_edge->setVertex(0, teb_.PoseVertex(i));
      velocity_edge->setVertex(1, teb_.PoseVertex(i + 1));
      velocity_edge->setVertex(2, teb_.TimeDiffVertex(i));
      velocity_edge->setInformation(information);
      velocity_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(velocity_edge);
    }
  } else  // holonomic-robot
  {
    if (cfg_->optim.weight_max_vel_x == 0 &&
        cfg_->optim.weight_max_vel_y == 0 &&
        cfg_->optim.weight_max_vel_theta == 0)
      return;  // if weight equals zero skip adding edges!

    int n = teb_.sizePoses();
    Eigen::Matrix<double, 3, 3> information;
    information.fill(0);
    information(0, 0) = cfg_->optim.weight_max_vel_x;
    information(1, 1) = cfg_->optim.weight_max_vel_y;
    information(2, 2) = cfg_->optim.weight_max_vel_theta;

    for (int i = 0; i < n - 1; ++i) {
      EdgeVelocityHolonomic* velocity_edge = new EdgeVelocityHolonomic;
      velocity_edge->setVertex(0, teb_.PoseVertex(i));
      velocity_edge->setVertex(1, teb_.PoseVertex(i + 1));
      velocity_edge->setVertex(2, teb_.TimeDiffVertex(i));
      velocity_edge->setInformation(information);
      velocity_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(velocity_edge);
    }
  }
}


void TebOptimalPlanner::AddEdgesVelocityCeres() {
  if (cfg_->robot.max_vel_y == 0)  // non-holonomic robot
  {
    if (cfg_->optim.weight_max_vel_x == 0 &&
        cfg_->optim.weight_max_vel_theta == 0)
      return;  // if weight equals zero skip adding edges!

    int n = teb_.sizePoses();
    Eigen::Matrix<double, 2, 2> information;
    information(0, 0) = cfg_->optim.weight_max_vel_x;
    information(1, 1) = cfg_->optim.weight_max_vel_theta;
    information(0, 1) = 0.0;
    information(1, 0) = 0.0;

    const Eigen::Matrix2d sqrt_information= information.llt().matrixL();

    for (int i = 0; i < n - 1; ++i) {
      ceres::CostFunction* cost_function =
          new ceres::AutoDiffCostFunction<ceres::CeresTebVelocity, 2, 1, 1, 1,
                                          1, 1, 1, 1>(
              new ceres::CeresTebVelocity(sqrt_information));

      problem.AddResidualBlock(
          cost_function, nullptr, &teb_.PoseVertex(i)->x(),
          &teb_.PoseVertex(i)->y(), &teb_.PoseVertex(i)->theta(),
          &teb_.PoseVertex(i + 1)->x(), &teb_.PoseVertex(i + 1)->y(),
          &teb_.PoseVertex(i + 1)->theta(), &teb_.TimeDiffVertex(i)->dt());
    }

    // for (int i = 0; i < n - 1; ++i) {
    //   EdgeVelocity* velocity_edge = new EdgeVelocity;
    //   velocity_edge->setVertex(0, teb_.PoseVertex(i));
    //   velocity_edge->setVertex(1, teb_.PoseVertex(i + 1));
    //   velocity_edge->setVertex(2, teb_.TimeDiffVertex(i));
    //   velocity_edge->setInformation(information);
    //   velocity_edge->setTebConfig(*cfg_);
    //   optimizer_->addEdge(velocity_edge);
    // }
  }
}

void TebOptimalPlanner::AddEdgesAcceleration() {
  if (cfg_->optim.weight_acc_lim_x == 0 &&
      cfg_->optim.weight_acc_lim_theta == 0)
    return;  // if weight equals zero skip adding edges!

  int n = teb_.sizePoses();

  if (cfg_->robot.max_vel_y == 0 ||
      cfg_->robot.acc_lim_y == 0)  // non-holonomic robot
  {
    Eigen::Matrix<double, 2, 2> information;
    information.fill(0);
    information(0, 0) = cfg_->optim.weight_acc_lim_x;
    information(1, 1) = cfg_->optim.weight_acc_lim_theta;

    // check if an initial velocity should be taken into accound
    if (vel_start_.first) {
      EdgeAccelerationStart* acceleration_edge = new EdgeAccelerationStart;
      acceleration_edge->setVertex(0, teb_.PoseVertex(0));
      acceleration_edge->setVertex(1, teb_.PoseVertex(1));
      acceleration_edge->setVertex(2, teb_.TimeDiffVertex(0));
      acceleration_edge->setInitialVelocity(vel_start_.second);
      acceleration_edge->setInformation(information);
      acceleration_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }

    // now add the usual acceleration edge for each tuple of three teb poses
    for (int i = 0; i < n - 2; ++i) {
      EdgeAcceleration* acceleration_edge = new EdgeAcceleration;
      acceleration_edge->setVertex(0, teb_.PoseVertex(i));
      acceleration_edge->setVertex(1, teb_.PoseVertex(i + 1));
      acceleration_edge->setVertex(2, teb_.PoseVertex(i + 2));
      acceleration_edge->setVertex(3, teb_.TimeDiffVertex(i));
      acceleration_edge->setVertex(4, teb_.TimeDiffVertex(i + 1));
      acceleration_edge->setInformation(information);
      acceleration_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }

    // check if a goal velocity should be taken into accound
    if (vel_goal_.first) {
      EdgeAccelerationGoal* acceleration_edge = new EdgeAccelerationGoal;
      acceleration_edge->setVertex(0, teb_.PoseVertex(n - 2));
      acceleration_edge->setVertex(1, teb_.PoseVertex(n - 1));
      acceleration_edge->setVertex(
          2, teb_.TimeDiffVertex(teb_.sizeTimeDiffs() - 1));
      acceleration_edge->setGoalVelocity(vel_goal_.second);
      acceleration_edge->setInformation(information);
      acceleration_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }
  } else  // holonomic robot
  {
    Eigen::Matrix<double, 3, 3> information;
    information.fill(0);
    information(0, 0) = cfg_->optim.weight_acc_lim_x;
    information(1, 1) = cfg_->optim.weight_acc_lim_y;
    information(2, 2) = cfg_->optim.weight_acc_lim_theta;

    // check if an initial velocity should be taken into accound
    if (vel_start_.first) {
      EdgeAccelerationHolonomicStart* acceleration_edge =
          new EdgeAccelerationHolonomicStart;
      acceleration_edge->setVertex(0, teb_.PoseVertex(0));
      acceleration_edge->setVertex(1, teb_.PoseVertex(1));
      acceleration_edge->setVertex(2, teb_.TimeDiffVertex(0));
      acceleration_edge->setInitialVelocity(vel_start_.second);
      acceleration_edge->setInformation(information);
      acceleration_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }

    // now add the usual acceleration edge for each tuple of three teb poses
    for (int i = 0; i < n - 2; ++i) {
      EdgeAccelerationHolonomic* acceleration_edge =
          new EdgeAccelerationHolonomic;
      acceleration_edge->setVertex(0, teb_.PoseVertex(i));
      acceleration_edge->setVertex(1, teb_.PoseVertex(i + 1));
      acceleration_edge->setVertex(2, teb_.PoseVertex(i + 2));
      acceleration_edge->setVertex(3, teb_.TimeDiffVertex(i));
      acceleration_edge->setVertex(4, teb_.TimeDiffVertex(i + 1));
      acceleration_edge->setInformation(information);
      acceleration_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }

    // check if a goal velocity should be taken into accound
    if (vel_goal_.first) {
      EdgeAccelerationHolonomicGoal* acceleration_edge =
          new EdgeAccelerationHolonomicGoal;
      acceleration_edge->setVertex(0, teb_.PoseVertex(n - 2));
      acceleration_edge->setVertex(1, teb_.PoseVertex(n - 1));
      acceleration_edge->setVertex(
          2, teb_.TimeDiffVertex(teb_.sizeTimeDiffs() - 1));
      acceleration_edge->setGoalVelocity(vel_goal_.second);
      acceleration_edge->setInformation(information);
      acceleration_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }
  }
}

void TebOptimalPlanner::AddEdgesAccelerationCeres() {
  if (cfg_->optim.weight_acc_lim_x == 0 &&
      cfg_->optim.weight_acc_lim_theta == 0)
    return;  // if weight equals zero skip adding edges!

  int n = teb_.sizePoses();

  if (cfg_->robot.max_vel_y == 0 ||
      cfg_->robot.acc_lim_y == 0)  // non-holonomic robot
  {
    Eigen::Matrix<double, 2, 2> information;
    information.fill(0);
    information(0, 0) = cfg_->optim.weight_acc_lim_x;
    information(1, 1) = cfg_->optim.weight_acc_lim_theta;

    const Eigen::Matrix2d sqrt_information= information.llt().matrixL();

    // check if an initial velocity should be taken into accound
    if (vel_start_.first) {
      // 这里实际考虑到了
      ceres::CostFunction* cost_function =
          new ceres::AutoDiffCostFunction<ceres::CeresTebAccelerationStart, 2,
                                          1, 1, 1, 1, 1, 1, 1>(
              new ceres::CeresTebAccelerationStart(sqrt_information,vel_start_.second));

      problem.AddResidualBlock(
          cost_function, nullptr, &teb_.PoseVertex(0)->x(),
          &teb_.PoseVertex(0)->y(), &teb_.PoseVertex(0)->theta(),
          &teb_.PoseVertex(1)->x(), &teb_.PoseVertex(1)->y(),
          &teb_.PoseVertex(1)->theta(), &teb_.TimeDiffVertex(0)->dt());

      // EdgeAccelerationStart* acceleration_edge = new EdgeAccelerationStart;
      // acceleration_edge->setVertex(0, teb_.PoseVertex(0));
      // acceleration_edge->setVertex(1, teb_.PoseVertex(1));
      // acceleration_edge->setVertex(2, teb_.TimeDiffVertex(0));
      // acceleration_edge->setInitialVelocity(vel_start_.second);
      // acceleration_edge->setInformation(information);
      // acceleration_edge->setTebConfig(*cfg_);
      // optimizer_->addEdge(acceleration_edge);
    }

    // now add the usual acceleration edge for each tuple of three teb poses
    for (int i = 0; i < n - 2; ++i) {
      ceres::CostFunction* cost_function =
          new ceres::AutoDiffCostFunction<ceres::CeresTebAcceleration, 2, 1, 1, 1,
                                          1, 1, 1, 1, 1, 1, 1, 1>(
              new ceres::CeresTebAcceleration(sqrt_information));

      problem.AddResidualBlock(
          cost_function, nullptr, &teb_.PoseVertex(i)->x(),
          &teb_.PoseVertex(i)->y(), &teb_.PoseVertex(i)->theta(),
          &teb_.PoseVertex(i + 1)->x(), &teb_.PoseVertex(i + 1)->y(),
          &teb_.PoseVertex(i + 1)->theta(), &teb_.PoseVertex(i + 2)->x(),
          &teb_.PoseVertex(i + 2)->y(), &teb_.PoseVertex(i + 2)->theta(),
          &teb_.TimeDiffVertex(i)->dt(), &teb_.TimeDiffVertex(i + 1)->dt());
    }

    // check if a goal velocity should be taken into accound
    if (vel_goal_.first) {
      // 这里实际也会执行
      ceres::CostFunction* cost_function =
          new ceres::AutoDiffCostFunction<ceres::CeresTebAccelerationGoal, 2,
                                          1, 1, 1, 1, 1, 1, 1>(
              new ceres::CeresTebAccelerationGoal(sqrt_information,vel_goal_.second));

      problem.AddResidualBlock(
          cost_function, nullptr, &teb_.PoseVertex(n-2)->x(),
          &teb_.PoseVertex(n-2)->y(), &teb_.PoseVertex(n-2)->theta(),
          &teb_.PoseVertex(n-1)->x(), &teb_.PoseVertex(n-1)->y(),
          &teb_.PoseVertex(n-1)->theta(), &teb_.TimeDiffVertex(teb_.sizeTimeDiffs()-1)->dt());
      // EdgeAccelerationGoal* acceleration_edge = new EdgeAccelerationGoal;
      // acceleration_edge->setVertex(0, teb_.PoseVertex(n - 2));
      // acceleration_edge->setVertex(1, teb_.PoseVertex(n - 1));
      // acceleration_edge->setVertex( 2, teb_.TimeDiffVertex(teb_.sizeTimeDiffs() - 1));
      // acceleration_edge->setGoalVelocity(vel_goal_.second);
      // acceleration_edge->setInformation(information);
      // acceleration_edge->setTebConfig(*cfg_);
      // optimizer_->addEdge(acceleration_edge);
    }
  }
}

void TebOptimalPlanner::AddEdgesTimeOptimal() {
  if (cfg_->optim.weight_optimaltime == 0)
    return;  // if weight equals zero skip adding edges!

  Eigen::Matrix<double, 1, 1> information;
  information.fill(cfg_->optim.weight_optimaltime);

  for (int i = 0; i < teb_.sizeTimeDiffs(); ++i) {
    EdgeTimeOptimal* timeoptimal_edge = new EdgeTimeOptimal;
    timeoptimal_edge->setVertex(0, teb_.TimeDiffVertex(i));
    timeoptimal_edge->setInformation(information);
    timeoptimal_edge->setTebConfig(*cfg_);
    optimizer_->addEdge(timeoptimal_edge);
  }
}

void TebOptimalPlanner::AddEdgesTimeOptimalCeres() {
  if (cfg_->optim.weight_optimaltime == 0)
    return;  // if weight equals zero skip adding edges!

  Eigen::Matrix<double, 1, 1> information;
  information.fill(cfg_->optim.weight_optimaltime);


  const Eigen::Matrix<double,1,1> sqrt_information = information.llt().matrixL();

  for (int i = 0; i < teb_.sizeTimeDiffs(); ++i) {
      ceres::CostFunction* cost_function =
          new ceres::AutoDiffCostFunction<ceres::CeresTebTimeOptimal, 1, 1 >(
              new ceres::CeresTebTimeOptimal(sqrt_information));

      problem.AddResidualBlock(cost_function, nullptr,
                               &teb_.TimeDiffVertex(i)->dt());
  }
}

void TebOptimalPlanner::AddEdgesShortestPath() {
  if (cfg_->optim.weight_shortest_path == 0)
    return;  // if weight equals zero skip adding edges!

  Eigen::Matrix<double, 1, 1> information;
  information.fill(cfg_->optim.weight_shortest_path);

  for (int i = 0; i < teb_.sizePoses() - 1; ++i) {
    EdgeShortestPath* shortest_path_edge = new EdgeShortestPath;
    shortest_path_edge->setVertex(0, teb_.PoseVertex(i));
    shortest_path_edge->setVertex(1, teb_.PoseVertex(i + 1));
    shortest_path_edge->setInformation(information);
    shortest_path_edge->setTebConfig(*cfg_);
    optimizer_->addEdge(shortest_path_edge);
  }
}

void TebOptimalPlanner::AddEdgesShortestPathCeres() {
  if (cfg_->optim.weight_shortest_path == 0)
    return;  // if weight equals zero skip adding edges!

  Eigen::Matrix<double, 1, 1> information;
  information.fill(cfg_->optim.weight_shortest_path);

  const Eigen::Matrix<double,1,1> sqrt_information = information.llt().matrixL();

  for (int i = 0; i < teb_.sizePoses() - 1; ++i) {
    ceres::CostFunction* cost_function =
        new ceres::AutoDiffCostFunction<ceres::CeresTebShortestPath, 1, 1, 1, 1,
                                        1>(
            new ceres::CeresTebShortestPath(sqrt_information));

    problem.AddResidualBlock(cost_function, nullptr,
                             &teb_.PoseVertex(i)->x(),&teb_.PoseVertex(i)->y(),
                             &teb_.PoseVertex(i+1)->x(),&teb_.PoseVertex(i+1)->y()
                             );
    // EdgeShortestPath* shortest_path_edge = new EdgeShortestPath;
    // shortest_path_edge->setVertex(0, teb_.PoseVertex(i));
    // shortest_path_edge->setVertex(1, teb_.PoseVertex(i + 1));
    // shortest_path_edge->setInformation(information);
    // shortest_path_edge->setTebConfig(*cfg_);
    // optimizer_->addEdge(shortest_path_edge);
  }
}

void TebOptimalPlanner::AddEdgesKinematicsDiffDrive() {
  if (cfg_->optim.weight_kinematics_nh == 0 &&
      cfg_->optim.weight_kinematics_forward_drive == 0)
    return;  // if weight equals zero skip adding edges!

  // create edge for satisfiying kinematic constraints
  Eigen::Matrix<double, 2, 2> information_kinematics;
  information_kinematics.fill(0.0);
  information_kinematics(0, 0) = cfg_->optim.weight_kinematics_nh;
  information_kinematics(1, 1) = cfg_->optim.weight_kinematics_forward_drive;

  for (int i = 0; i < teb_.sizePoses() - 1; i++)  // ignore twiced start only
  {
    EdgeKinematicsDiffDrive* kinematics_edge = new EdgeKinematicsDiffDrive;
    kinematics_edge->setVertex(0, teb_.PoseVertex(i));
    kinematics_edge->setVertex(1, teb_.PoseVertex(i + 1));
    kinematics_edge->setInformation(information_kinematics);
    kinematics_edge->setTebConfig(*cfg_);
    optimizer_->addEdge(kinematics_edge);
  }
}

void TebOptimalPlanner::AddEdgesKinematicsDiffDriveCeres() {
  if (cfg_->optim.weight_kinematics_nh == 0 &&
      cfg_->optim.weight_kinematics_forward_drive == 0)
    return;  // if weight equals zero skip adding edges!

  // create edge for satisfiying kinematic constraints
  Eigen::Matrix<double, 2, 2> information_kinematics;
  information_kinematics.fill(0.0);
  information_kinematics(0, 0) = cfg_->optim.weight_kinematics_nh;
  information_kinematics(1, 1) = cfg_->optim.weight_kinematics_forward_drive;

  const Eigen::Matrix<double,2,2> sqrt_information = information_kinematics.llt().matrixL();

  for (int i = 0; i < teb_.sizePoses() - 1; i++)  // ignore twiced start only
  {
    // EdgeKinematicsDiffDrive* kinematics_edge = new EdgeKinematicsDiffDrive;
    // kinematics_edge->setVertex(0, teb_.PoseVertex(i));
    // kinematics_edge->setVertex(1, teb_.PoseVertex(i + 1));
    // kinematics_edge->setInformation(information_kinematics);
    // kinematics_edge->setTebConfig(*cfg_);
    // optimizer_->addEdge(kinematics_edge);
    ceres::CostFunction* cost_function =
        new ceres::AutoDiffCostFunction<ceres::CeresTebKinematicsDiffDriveCeres,
                                        2, 1, 1, 1, 1, 1, 1>(
            new ceres::CeresTebKinematicsDiffDriveCeres(sqrt_information));

    problem.AddResidualBlock(
        cost_function, nullptr, &teb_.PoseVertex(i)->x(),
        &teb_.PoseVertex(i)->y(), &teb_.PoseVertex(i)->theta(),
        &teb_.PoseVertex(i + 1)->x(), &teb_.PoseVertex(i + 1)->y(),
        &teb_.PoseVertex(i + 1)->theta());
  }
}

void TebOptimalPlanner::AddEdgesKinematicsCarlike() {
  if (cfg_->optim.weight_kinematics_nh == 0 &&
      cfg_->optim.weight_kinematics_turning_radius == 0)
    return;  // if weight equals zero skip adding edges!

  // create edge for satisfiying kinematic constraints
  Eigen::Matrix<double, 2, 2> information_kinematics;
  information_kinematics.fill(0.0);
  information_kinematics(0, 0) = cfg_->optim.weight_kinematics_nh;
  information_kinematics(1, 1) = cfg_->optim.weight_kinematics_turning_radius;

  for (int i = 0; i < teb_.sizePoses() - 1; i++)  // ignore twiced start only
  {
    EdgeKinematicsCarlike* kinematics_edge = new EdgeKinematicsCarlike;
    kinematics_edge->setVertex(0, teb_.PoseVertex(i));
    kinematics_edge->setVertex(1, teb_.PoseVertex(i + 1));
    kinematics_edge->setInformation(information_kinematics);
    kinematics_edge->setTebConfig(*cfg_);
    optimizer_->addEdge(kinematics_edge);
  }
}

void TebOptimalPlanner::AddEdgesPreferRotDir() {
  // TODO(roesmann): Note, these edges can result in odd predictions, in
  // particular
  //                 we can observe a substantional mismatch between open- and
  //                 closed-loop planning leading to a poor control performance.
  //                 At the moment, we keep these functionality for oscillation
  //                 recovery: Activating the edge for a short time period might
  //                 not be crucial and could move the robot to a new
  //                 oscillation-free state. This needs to be analyzed in more
  //                 detail!
  if (prefer_rotdir_ == RotType::none || cfg_->optim.weight_prefer_rotdir == 0)
    return;  // if weight equals zero skip adding edges!

  if (prefer_rotdir_ != RotType::right && prefer_rotdir_ != RotType::left) {
    // ROS_WARN("TebOptimalPlanner::AddEdgesPreferRotDir(): unsupported RotType
    // selected. Skipping edge creation.");
    return;
  }

  // create edge for satisfiying kinematic constraints
  Eigen::Matrix<double, 1, 1> information_rotdir;
  information_rotdir.fill(cfg_->optim.weight_prefer_rotdir);

  for (int i = 0; i < teb_.sizePoses() - 1 && i < 3;
       ++i)  // currently: apply to first 3 rotations
  {
    EdgePreferRotDir* rotdir_edge = new EdgePreferRotDir;
    rotdir_edge->setVertex(0, teb_.PoseVertex(i));
    rotdir_edge->setVertex(1, teb_.PoseVertex(i + 1));
    rotdir_edge->setInformation(information_rotdir);

    if (prefer_rotdir_ == RotType::left)
      rotdir_edge->preferLeft();
    else if (prefer_rotdir_ == RotType::right)
      rotdir_edge->preferRight();

    optimizer_->addEdge(rotdir_edge);
  }
}

void TebOptimalPlanner::computeCurrentCost(double obst_cost_scale,
                                           double viapoint_cost_scale,
                                           bool alternative_time_cost) {
  // check if graph is empty/exist  -> important if function is called between
  // buildGraph and optimizeGraph/clearGraph
  bool graph_exist_flag(false);
  if (optimizer_->edges().empty() && optimizer_->vertices().empty()) {
    // here the graph is build again, for time efficiency make sure to call this
    // function between buildGraph and Optimize (deleted), but it depends on the
    // application
    buildGraph();
    optimizer_->initializeOptimization();
    gDebugCol3("debug")<<FILE_LINE;
  } else {
    graph_exist_flag = true;
  }

  optimizer_->computeInitialGuess();

  cost_ = 0;

  if (alternative_time_cost) {
    cost_ += teb_.getSumOfAllTimeDiffs();
    // TEST we use SumOfAllTimeDiffs() here, because edge cost depends on number
    // of samples, which is not always the same for similar TEBs, since we are
    // using an AutoResize Function with hysteresis.
  }

  // now we need pointers to all edges -> calculate error for each edge-type
  // since we aren't storing edge pointers, we need to check every edge
  for (std::vector<g2o::OptimizableGraph::Edge*>::const_iterator it =
           optimizer_->activeEdges().begin();
       it != optimizer_->activeEdges().end(); it++) {
    double cur_cost = (*it)->chi2();

    if (dynamic_cast<EdgeObstacle*>(*it) != nullptr ||
        dynamic_cast<EdgeInflatedObstacle*>(*it) != nullptr ||
        dynamic_cast<EdgeDynamicObstacle*>(*it) != nullptr) {
      cur_cost *= obst_cost_scale;
    } else if (dynamic_cast<EdgeViaPoint*>(*it) != nullptr) {
      cur_cost *= viapoint_cost_scale;
    } else if (dynamic_cast<EdgeTimeOptimal*>(*it) != nullptr &&
               alternative_time_cost) {
      continue;  // skip these edges if alternative_time_cost is active
    }
    cost_ += cur_cost;
  }

  // delete temporary created graph
  if (!graph_exist_flag) clearGraph();
}

void TebOptimalPlanner::extractVelocity(const PoseSE2& pose1,
                                        const PoseSE2& pose2, double dt,
                                        float& vx, float& vy,
                                        float& omega) const {
  if (dt == 0) {
    vx = 0;
    vy = 0;
    omega = 0;
    return;
  }

  Eigen::Vector2d deltaS = pose2.position() - pose1.position();

  if (cfg_->robot.max_vel_y == 0)  // nonholonomic robot
  {
    Eigen::Vector2d conf1dir(cos(pose1.theta()), sin(pose1.theta()));
    // translational velocity
    double dir = deltaS.dot(conf1dir);
    vx = (double)g2o::sign(dir) * deltaS.norm() / dt;
    vy = 0;
  } else  // holonomic robot
  {
    // transform pose 2 into the current robot frame (pose1)
    // for velocities only the rotation of the direction vector is necessary.
    // (map->pose1-frame: inverse 2d rotation matrix)
    double cos_theta1 = std::cos(pose1.theta());
    double sin_theta1 = std::sin(pose1.theta());
    double p1_dx = cos_theta1 * deltaS.x() + sin_theta1 * deltaS.y();
    double p1_dy = -sin_theta1 * deltaS.x() + cos_theta1 * deltaS.y();
    vx = p1_dx / dt;
    vy = p1_dy / dt;
  }

  // rotational velocity
  double orientdiff = g2o::normalize_theta(pose2.theta() - pose1.theta());
  omega = orientdiff / dt;
}

bool TebOptimalPlanner::getVelocityCommand(float& vx, float& vy, float& omega,
                                           int look_ahead_poses) const {
  if (teb_.sizePoses() < 2) {
    // ROS_ERROR("TebOptimalPlanner::getVelocityCommand(): The trajectory
    // contains less than 2 poses. Make sure to init and optimize/plan the
    // trajectory fist.");
    vx = 0;
    vy = 0;
    omega = 0;
    return false;
  }
  look_ahead_poses =
      std::max(1, std::min(look_ahead_poses, teb_.sizePoses() - 1));
  double dt = 0.0;
  for (int counter = 0; counter < look_ahead_poses; ++counter) {
    dt += teb_.TimeDiff(counter);
    if (dt >= cfg_->trajectory.dt_ref *
                  look_ahead_poses)  // TODO: change to look-ahead time? Refine
                                     // trajectory?
    {
      look_ahead_poses = counter + 1;
      break;
    }
  }
  if (dt <= 0) {
    // ROS_ERROR("TebOptimalPlanner::getVelocityCommand() - timediff<=0 is
    // invalid!");
    vx = 0;
    vy = 0;
    omega = 0;
    return false;
  }

  // Get velocity from the first two configurations
  extractVelocity(teb_.Pose(0), teb_.Pose(look_ahead_poses), dt, vx, vy, omega);
  return true;
}

void TebOptimalPlanner::getVelocityProfile(
    std::vector<Twist>& velocity_profile) const {
  int n = teb_.sizePoses();
  velocity_profile.resize(n + 1);

  // start velocity
  velocity_profile.front().linear.z() = 0;
  velocity_profile.front().angular.x() = velocity_profile.front().angular.y() =
      0;
  velocity_profile.front().linear.x() = vel_start_.second.linear.x();
  velocity_profile.front().linear.y() = vel_start_.second.linear.y();
  velocity_profile.front().angular.z() = vel_start_.second.angular.z();

  for (int i = 1; i < n; ++i) {
    velocity_profile[i].linear.z() = 0;
    velocity_profile[i].angular.x() = velocity_profile[i].angular.y() = 0;
    extractVelocity(teb_.Pose(i - 1), teb_.Pose(i), teb_.TimeDiff(i - 1),
                    velocity_profile[i].linear.x(),
                    velocity_profile[i].linear.y(),
                    velocity_profile[i].angular.z());
  }

  // goal velocity
  velocity_profile.back().linear.z() = 0;
  velocity_profile.back().angular.x() = velocity_profile.back().angular.y() = 0;
  velocity_profile.back().linear.x() = vel_goal_.second.linear.x();
  velocity_profile.back().linear.y() = vel_goal_.second.linear.y();
  velocity_profile.back().angular.z() = vel_goal_.second.angular.z();
}
void TebOptimalPlanner::getFullTrajectory(
    std::vector<Eigen::Vector3f>& trajectory) const {
  int n = teb_.sizePoses();
  trajectory.resize(n);

  if (n == 0) return;

  double curr_time = 0;

  // start
  Eigen::Vector3f& start = trajectory.front();
  start[0] = teb_.Pose(0).x();
  start[1] = teb_.Pose(0).y();
  start[2] = teb_.Pose(0).theta();

  curr_time += teb_.TimeDiff(0);

  // intermediate points
  for (int i = 1; i < n - 1; ++i) {
    Eigen::Vector3f& point = trajectory[i];
    point[0] = teb_.Pose(i).x();
    point[1] = teb_.Pose(i).y();
    point[2] = teb_.Pose(i).theta();
    curr_time += teb_.TimeDiff(i);
  }

  // goal
  Eigen::Vector3f& goal = trajectory.back();
  goal[0] = teb_.Pose(n - 1).x();
  goal[1] = teb_.Pose(n - 1).y();
  goal[2] = teb_.Pose(n - 1).theta();
}

//    void TebOptimalPlanner::getFullTrajectory(std::vector<TrajectoryPointMsg>&
//    trajectory) const
//    {
//        int n = teb_.sizePoses();
//
//        trajectory.resize(n);
//
//        if (n == 0)
//            return;
//
//        double curr_time = 0;
//
//        // start
//        TrajectoryPointMsg& start = trajectory.front();
//        teb_.Pose(0).toPoseMsg(start.pose);
//        start.velocity.linear.z = 0;
//        start.velocity.angular.x = start.velocity.angular.y = 0;
//        start.velocity.linear.x = vel_start_.second.linear.x;
//        start.velocity.linear.y = vel_start_.second.linear.y;
//        start.velocity.angular.z = vel_start_.second.angular.z;
//        start.time_from_start.fromSec(curr_time);
//
//        curr_time += teb_.TimeDiff(0);
//
//        // intermediate points
//        for (int i=1; i < n-1; ++i)
//        {
//            TrajectoryPointMsg& point = trajectory[i];
//            teb_.Pose(i).toPoseMsg(point.pose);
//            point.velocity.linear.z = 0;
//            point.velocity.angular.x = point.velocity.angular.y = 0;
//            double vel1_x, vel1_y, vel2_x, vel2_y, omega1, omega2;
//            extractVelocity(teb_.Pose(i-1), teb_.Pose(i), teb_.TimeDiff(i-1),
//            vel1_x, vel1_y, omega1); extractVelocity(teb_.Pose(i),
//            teb_.Pose(i+1), teb_.TimeDiff(i), vel2_x, vel2_y, omega2);
//            point.velocity.linear.x = 0.5*(vel1_x+vel2_x);
//            point.velocity.linear.y = 0.5*(vel1_y+vel2_y);
//            point.velocity.angular.z = 0.5*(omega1+omega2);
//            point.time_from_start.fromSec(curr_time);
//
//            curr_time += teb_.TimeDiff(i);
//        }
//
//        // goal
//        TrajectoryPointMsg& goal = trajectory.back();
//        teb_.BackPose().toPoseMsg(goal.pose);
//        goal.velocity.linear.z = 0;
//        goal.velocity.angular.x = goal.velocity.angular.y = 0;
//        goal.velocity.linear.x = vel_goal_.second.linear.x;
//        goal.velocity.linear.y = vel_goal_.second.linear.y;
//        goal.velocity.angular.z = vel_goal_.second.angular.z;
//        goal.time_from_start.fromSec(curr_time);
//    }
//

bool TebOptimalPlanner::isTrajectoryFeasible(void) { return true; }

}  // namespace teb_local_planner
