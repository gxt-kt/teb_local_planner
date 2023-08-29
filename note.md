
1. std::unique_ptr<TEBLinearSolver> linear_solver( new TEBLinearSolver());  // see typedef in optimization.h
  what is `TEBLinearSolver`

2. typedef g2o::LinearSolverCSparse<TEBBlockSolver::PoseMatrixType> TEBLinearSolver;
  what is TEBBlockSolver and PoseMatrixType

3. typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> >  TEBBlockSolver;
   typedef typename Traits::PoseMatrixType PoseMatrixType; // Traits here is g2o::BlockSolverTraits<-1, -1>
   what is g2o::BlockSolverTraits<-1, -1>::PoseMatrixType

4. typedef MatrixX PoseMatrixType; // in class BlockSolverTraits<Eigen::Dynamic, Eigen::Dynamic>
   what is MatrixX

5. using MatrixX = MatrixN<Eigen::Dynamic>;

So the TEBLinearSolver is :
g2o::LinearSolverCSparse<g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> >::MatrixN<Eigen::Dynamic>
g2o::LinearSolverCSparse<g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> >::MatrixN<Eigen::Dynamic> >


### 1. 配置优化器

```cpp
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
```




## GXT

### include g2o lib files

- ./inc/g2o_types/*
- ./inc/optimal_planner.h
- ./inc/timed_elastic_band.h
- ./inc/visualization.h (not important, this files just include timed_elastic_band.h)
### class

PoseSE2: (use to express a statue)
  - Eigen::Vector2d _position;
  - double _theta;


VertexPose : public g2o::BaseVertex<3, PoseSE2 >


template <int D, typename E, typename VertexXi>
class BaseTebUnaryEdge : public g2o::BaseUnaryEdge<D, E, VertexXi> {
  const TebConfig* cfg_; //!< Store TebConfig class for parameters
}



template <int D, typename E, typename VertexXi, typename VertexXj>
class BaseTebBinaryEdge : public g2o::BaseBinaryEdge<D, E, VertexXi, VertexXj> {
  const TebConfig* cfg_; //!< Store TebConfig class for parameters
}


template <int D, typename E>
class BaseTebMultiEdge : public g2o::BaseMultiEdge<D, E> {
  const TebConfig* cfg_; //!< Store TebConfig class for parameters
}


class **EdgeAcceleration** : public BaseTebMultiEdge<2, double> (restrict translation and rotation acceleration)


typedef boost::shared_ptr<Obstacle> ObstaclePtr;
class Obstacle {
    bool dynamic_; //!< Store flag if obstacle is dynamic (resp. a moving obstacle)
    Eigen::Vector2d centroid_velocity_; //!< Store the corresponding velocity (vx, vy) of the centroid (zero, if _dynamic is \c true)
}

typedef std::vector<Eigen::Vector2d> ViaPointContainer;
ViaPointContainer via_points;
via_points.push_back(Eigen::Vector2d(1,6));


typedef boost::shared_ptr<BaseRobotFootprintModel> RobotFootprintModelPtr;
class CircularRobotFootprint : public BaseRobotFootprintModel {
    double radius_;
}
RobotFootprintModelPtr robot_model = boost::make_shared<CircularRobotFootprint>(0.3);



## 初始化

1. 比如一个起始点，一个终点，会补充到3（min_sample）个点，也就是中间差值一个

2. 然后再次插值，保证每两个点之间的timediff小于一定0.4和大于0.2，并总点数小于500。

   - timediff根据两个位姿的距离/max_vel
   - 大于0.4就在中间差值
   - 小于0.2就把点删了

3. 根据上面的差值，最后位姿点数比时间差数刚好大一个，比如

   teb_.sizePoses() = 65 
   teb_.sizeTimeDiffs() = 64 

4. 图优化添加顶点：交替添加位姿和时间差，比如先添加位姿id=0，添加时间差id=1，在添加位姿id=2，再添加时间差id=3，由于位姿数刚好比时间差多一个，所以最后添加的一个为位姿（id=128）

5. 图优化添加障碍物的边：

   - 有两种关于障碍物的算法（我们用的新的）
     - 旧的：对于每个障碍物，查找最近的 TEB 位姿
     - 新的：对于每个 teb 位姿，仅查找“相关”障碍物

   - 去除第一个和最后一个点，然后计算障碍物到顶点的距离

   - 如果距离很近，就认为是相关的障碍物边
   - 如果距离较远，就直接不考虑
   - 普通距离也会考虑一个最相邻左边和一个最相邻右边的障碍物
   - 然后添加相关障碍物边和一个最左边和最右边

6. 图优化添加经过点的边：对于每个经过点，找到和它最近的顶点，还要确保找到的顶点不会是开头和结束的点

   - 然后把经过点约束到最相邻顶点的边

7. 图优化添加机器人速度边约束：约束两个相邻顶点的速度，多元边，顶点是两个相邻位姿和对应的时间差，如果位姿数是65个，这里的边就是添加64条

8. 图优化添加机器人加速度约束：

   - 如果有初始速度vel_start_.first，也把初始速度加入约束
   - 然后根据三个相邻姿态点添加加速度约束，如果位姿数是65个，这里的边就是添加63条
   - 如果有最终目标点速度vel_end_.first，也把初始速度加入约束

9. 图优化添加时间约束：如果时间数是64个，这里的边就是添加64条

10. 图优化添加最短路径约束：和相邻两个位姿有关，让两个位姿距离最小，如果位姿数是65个，这里的边就是添加64条

11. 图优化添加差动驱动机器人的运动学约束：运动学约束，和相邻两个位姿有关，如果位姿数是65个，这里的边就是添加64条



## 倒车问题

和三个变量相关：

1. max_vel_x_backwards 最大倒车速度，这个值不能设置为0或者负数，否则不收敛，倒车问题不能通过这个变量解决

2. weight_kinematics_forward_drive 用于强制机器人仅选择前进方向的优化权重，可用于解决倒车问题，加大此变量，会加大倒车惩罚

   - ```cpp
     information_kinematics(1, 1) = cfg_->optim.weight_kinematics_forward_drive;
     
     const VertexPose* conf1 = static_cast<const VertexPose*>(_vertices[0]);
     const VertexPose* conf2 = static_cast<const VertexPose*>(_vertices[1]);
     
     Eigen::Vector2d deltaS = conf2->position() - conf1->position();
     
     // non holonomic constraint
     _error[0] = fabs( ( cos(conf1->theta())+cos(conf2->theta()) ) * deltaS[1] - ( sin(conf1->theta())+sin(conf2->theta()) ) * deltaS[0] );
     
     // positive-drive-direction constraint
     Eigen::Vector2d angle_vec ( cos(conf1->theta()), sin(conf1->theta()) );	   
     _error[1] = penaltyBoundFromBelow(deltaS.dot(angle_vec), 0,0);
     
     inline double penaltyBoundFromBelow(const double& var, const double& a,const double& epsilon)
     {
       if (var >= a+epsilon)
       {
         return 0.;
       }
       else
       {
         return (-var + (a+epsilon));
       }
     }
     ```

3. 和最优化时间weight_optimaltime相关

   如果把这个值设置成0，就会原地打转，再移动

   



## enable_homotopy_class_planning

在ros中，有两种优化方法，根据enable_homotopy_class_planning变量进行选择

1. HomotopyClassPlanner
2. 如果不开启就是HomotopyClassPlanner



### calculateHSignature

```cpp
template<typename BidirIter, typename Fun>
void calculateHSignature(BidirIter path_start, BidirIter path_end, Fun fun_cplx_point, const ObstContainer* obstacles)
```

假设障碍物尺寸3，m=5

a=3

b=2

把最后点移到倒数第二个

delta是start指向end的向量

normal是把delta逆时针旋转90度



***

### 

```cpp
bool HomotopyClassPlanner::plan(const PoseSE2& start, const PoseSE2& goal, const geometry_msgs::Twist* start_vel, bool free_goal_vel)
{
  ROS_ASSERT_MSG(initialized_, "Call initialize() first.");

  // Update old TEBs with new start, goal and velocity
  updateAllTEBs(&start, &goal, start_vel);

  // Init new TEBs based on newly explored homotopy classes
  exploreEquivalenceClassesAndInitTebs(start, goal, cfg_->obstacles.min_obstacle_dist, start_vel, free_goal_vel);
  // update via-points if activated
  updateReferenceTrajectoryViaPoints(cfg_->hcp.viapoints_all_candidates);
  // Optimize all trajectories in alternative homotopy classes
  optimizeAllTEBs(cfg_->optim.no_inner_iterations, cfg_->optim.no_outer_iterations);
  // Select which candidate (based on alternative homotopy classes) should be used
  selectBestTeb();

  initial_plan_ = nullptr; // clear pointer to any previous initial plan (any previous plan is useless regarding the h-signature);
  return true;
}
```

**plan** 函数：

- 通过 updateAllTEBs 更新当前所有路径的起点终点和速度
- 通过exploreEquivalenceClassesAndInitTebs对老路径进行一波筛选，再寻找新的路径
- updateReferenceTrajectoryViaPoints 配置经过点，暂时没有被用到
- optimizeAllTEBs 用多线程优化所有的teb路径





**updateAllTEBs** 的作用是更新当前所有teb路径的起点重点和起点速度：

- 如果新的终点离当前路径太远或者角度差太大（阈值分别为force_reinit_new_goal_dist和force_reinit_new_goal_angular），**就直接清空所有teb（准备从头开始）**
- 否则遍历当前所有的teb路径，通过函数force_reinit_new_goal_angular重新给到新起点和新终点和新的起点速度



**updateAndPruneTEB** 函数的作用

- 把起点移到距离新起点最近的一个点，然后把那个点设成新起点。然后把最后一个点设置成新的终点点



**exploreEquivalenceClassesAndInitTebs**的目标是探索新的同伦类

- 首先调用renewAndAnalyzeOldTebs函数，对旧的路径进行删选，比如去掉更新后同源的类和向后走时间长的类
- 会调用createGraph开始找新的路径



**renewAndAnalyzeOldTebs**函数：

- 会看当前存不存在一条最好路径

  - 如果存在最好路径，就把最佳路径交换放到第一个，然后计算出最好路径的特征向量，然后用addEquivalenceClassIfNew添加进去
  - 然后对剩下的路径进行遍历，删除掉同源的路径

  再看看是否删除掉不好的路径（偏差太大或者时间太长）？丢弃相对于最佳计划向后绕行的计划





**calculateEquivalenceClass**函数是计算teb路径的对应的特征向量



**addEquivalenceClassIfNew** 函数是根据判断**是否添加**新的相似特征向量

- 首先判断传入的相似特征向量是否有效，如果有效就看

  1.已经满了

  2.和最好的不接近

  两者有一个满足就不添加，两个都不满足才会添加这个特征向量

  添加了才会返回真



**randomlyDropTebs** 实际运行时没有用



## 代码移植，有哪些内容需要做 （移植这一部分已经完成）😀

homotopy_class_planner.h：225 函数 isTrajectoryFeasible 需要修改，isTrajectoryFeasible 函数是否被用到了 （看来这个函数并不会被用到）

时间问题：ros::Time 需要修改 （已经完成）

hasDiverged是否被使用了 （看来这个函数并不会被用到）


## zhangaiwu 残差 jisuan (以解决，使用ceres的数值求导)
robot_model:CircularRobotFootprint
obstacle:point obstacle
virtual double getMinimumDistance(const Eigen::Vector2d& position) const
{
    return (position-pos_).norm();
}

## 观察teb源码：拿到全局路径和代价地图
问题：拿到的代价地图是怎么转换成障碍物容器的
有一个配置 `cfg_.obstacles.costmap_converter_plugin`
如果为空，就是不启用插件，这样激光雷达的点就是障碍物，都是point类型的
如果启用，就用运行对应属性的插件，把costmap转换成障碍物
teb默认是没有启用插件的，也就是全都是point类型障碍物

```cpp
// reserve some memory for obstacles
obstacles_.reserve(500);
```

默认不用插件的源码
```cpp
void TebLocalPlannerROS::updateObstacleContainerWithCostmap()
{
// Add costmap obstacles if desired
if (cfg_.obstacles.include_costmap_obstacles)
{
Eigen::Vector2d robot_orient = robot_pose_.orientationUnitVec();

for (unsigned int i=0; i<costmap_->getSizeInCellsX()-1; ++i)  
{  
  for (unsigned int j=0; j<costmap_->getSizeInCellsY()-1; ++j)  
  {  
    if (costmap_->getCost(i,j) == costmap_2d::LETHAL_OBSTACLE)  
    {  
      Eigen::Vector2d obs;  
      costmap_->mapToWorld(i,j,obs.coeffRef(0), obs.coeffRef(1));  
          
      // check if obstacle is interesting (e.g. not far behind the robot)  
      Eigen::Vector2d obs_dir = obs-robot_pose_.position();  
      if ( obs_dir.dot(robot_orient) < 0 && obs_dir.norm() > cfg_.obstacles.costmap_obstacles_behind_robot_dist  )  
        continue;  
          
      obstacles_.push_back(ObstaclePtr(new PointObstacle(obs)));  
    }  
  }  
}  
}
}

```

