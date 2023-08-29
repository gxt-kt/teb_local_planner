#include "../teb_config.h"
#include "Eigen/Core"
#include "ceres/ceres.h"
#include "debugstream.hpp"
#include "g2o_types/penalties.h"
#include "teb_types.h"
#include "robot_footprint_model.h"
#include "obstacles.h"
#include "pose_se2.h"

// using namespace teb_local_planner;
extern teb_local_planner::TebConfig config;

namespace ceres {

template <typename T>
inline T penaltyBoundFromBelow(const T& var, const T& a,const T& epsilon)
{
  if (var >= a+epsilon)
  {
    return T(0);
  }
  else
  {
    return (-var + (a+epsilon));
  }
}

template <typename T>
inline T penaltyBoundToInterval(const T& var, const T& a, const T& epsilon) {
  if (var < -a + epsilon) {
    return (-var - (a - epsilon));
  }
  if (var <= a - epsilon) {
    return T(0);
  } else {
    return (var - (a - epsilon));
  }
}

template <typename T>
inline T penaltyBoundToInterval(const T& var, const T& a, const T& b,
                                const T& epsilon) {
  if (var < a + epsilon) {
    return (-var + (a + epsilon));
  }
  if (var <= b - epsilon) {
    return T(0);
  } else {
    return (var - (b - epsilon));
  }
}
};  // namespace ceres

namespace ceres {

template <typename T>
inline T const_pi() {
  return T(M_PI);
}
template <typename T>
inline T normalize_theta(T theta) {
  if (theta >= -const_pi<T>() && theta < const_pi<T>()) return theta;

  T multiplier = ceres::floor(theta / (T(2) * const_pi<T>()));
  theta = theta - multiplier * T(2) * const_pi<T>();
  if (theta >= const_pi<T>()) theta -= T(2) * const_pi<T>();
  if (theta < -const_pi<T>()) theta += T(2) * const_pi<T>();

  return theta;
}

template <typename T>
inline T fast_sigmoid(T x) {
  return x / (T(1) + ceres::abs(x));
}

class CeresTebVelocity {
 public:
  CeresTebVelocity(const Eigen::Matrix2d& sqrt_information)
      : sqrt_information_(sqrt_information) {}

  template <typename T>
  bool operator()(const T* const x_a, const T* const y_a, const T* const yaw_a,
                  const T* const x_b, const T* const y_b, const T* const yaw_b,
                  const T* const time_diff, T* residuals_ptr) const {
    Eigen::Map<Eigen::Matrix<T, 2, 1>> residuals_map(residuals_ptr);

    Eigen::Matrix<T, 2, 1> x1(*x_a, *y_a);
    Eigen::Matrix<T, 2, 1> x2(*x_b, *y_b);
    Eigen::Matrix<T, 2, 1> deltaS = x2 - x1;
    T dist = deltaS.norm();
    const T angle_diff = ceres::normalize_theta(*yaw_b - *yaw_a);
    if (config.trajectory.exact_arc_length && angle_diff != 0) {
      T radius = dist / (T(2) * sin(angle_diff / T(2)));
      dist = ceres::abs(angle_diff * radius);
    }
    T vel = dist / *time_diff;
    vel *= ceres::fast_sigmoid(T(100) * (deltaS(0) * ceres::cos(*yaw_a)) +
                               deltaS(1) * ceres::sin(*yaw_a));
    const T omega = angle_diff / *time_diff;

    residuals_map(0) = ceres::penaltyBoundToInterval<T>(
        vel, -T(config.robot.max_vel_x_backwards), T(config.robot.max_vel_x),
        T(config.optim.penalty_epsilon));
    residuals_map(1) = ceres::penaltyBoundToInterval<T>(
        omega, T(config.robot.max_vel_theta), T(config.optim.penalty_epsilon));

    residuals_map = sqrt_information_.template cast<T>() * residuals_map;

    return true;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  // The inverse square root of the measurement covariance matrix.
  const Eigen::Matrix2d sqrt_information_;
};


class CeresTebInflatedObstacle {
 public:
  CeresTebInflatedObstacle(const Eigen::Matrix2d& sqrt_information, const teb_local_planner::BaseRobotFootprintModel* robot_model, const teb_local_planner::Obstacle* obstacle)
      : sqrt_information_(sqrt_information),robot_model_(robot_model),_measurement(obstacle) {}

  template <typename T>
  bool operator()(const T* const x_a, const T* const y_a,T* residuals_ptr) const {
    Eigen::Map<Eigen::Matrix<T, 2, 1>> residuals_map(residuals_ptr);

    // teb_local_planner::PoseSE2 pose;
    // T dist111 = T(robot_model_->calculateDistance(pose, _measurement));

    auto pos = _measurement->getCentroid();
    Eigen::Vector<T,2> current_pose(*x_a,*y_a);
    Eigen::Vector<T,2> pos_(T(pos(0)),T(pos(1)));
    T dist=T((current_pose-pos_).norm())-T(0.15); //0.15 is radius_

    residuals_map(0) = ceres::penaltyBoundFromBelow(dist, T(config.obstacles.min_obstacle_dist),T(config.optim.penalty_epsilon));
    if (config.optim.obstacle_cost_exponent != 1.0 && config.obstacles.min_obstacle_dist > 0.0)
    {
      residuals_map(0) = config.obstacles.min_obstacle_dist * ceres::pow(residuals_map(0) / T(config.obstacles.min_obstacle_dist), T(config.optim.obstacle_cost_exponent));
    }

    residuals_map(1) = ceres::penaltyBoundFromBelow(dist, T(config.obstacles.inflation_dist),T(0));

    residuals_map = sqrt_information_.template cast<T>() * residuals_map;

    return true;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  // The inverse square root of the measurement covariance matrix.
  const Eigen::Matrix2d sqrt_information_;
  const teb_local_planner::BaseRobotFootprintModel* robot_model_; //!< Store pointer to robot_model
  const teb_local_planner::Obstacle* _measurement;
};


class CeresTebInflatedObstacleNumeric {
 public:
  CeresTebInflatedObstacleNumeric(const Eigen::Matrix2d& sqrt_information, const teb_local_planner::BaseRobotFootprintModel* robot_model, const teb_local_planner::Obstacle* obstacle)
      : sqrt_information_(sqrt_information),robot_model_(robot_model),_measurement(obstacle) {}

  bool operator()(const double* const x_a, const double* const y_a,double* residuals_ptr) const {
    Eigen::Map<Eigen::Matrix<double, 2, 1>> residuals_map(residuals_ptr);

    teb_local_planner::PoseSE2 pose;
    pose.x()=*x_a;
    pose.y()=*y_a;
    double dist = double(robot_model_->calculateDistance(pose, _measurement));


    residuals_map(0) = ceres::penaltyBoundFromBelow(dist, double(config.obstacles.min_obstacle_dist),double(config.optim.penalty_epsilon));
    if (config.optim.obstacle_cost_exponent != 1.0 && config.obstacles.min_obstacle_dist > 0.0)
    {
      residuals_map(0) = config.obstacles.min_obstacle_dist * ceres::pow(residuals_map(0) / double(config.obstacles.min_obstacle_dist), double(config.optim.obstacle_cost_exponent));
    }

    residuals_map(1) = ceres::penaltyBoundFromBelow(dist, double(config.obstacles.inflation_dist),double(0));

    residuals_map = sqrt_information_.template cast<double>() * residuals_map;

    return true;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  // The inverse square root of the measurement covariance matrix.
  const Eigen::Matrix2d sqrt_information_;
  const teb_local_planner::BaseRobotFootprintModel* robot_model_; //!< Store pointer to robot_model
  const teb_local_planner::Obstacle* _measurement;
};

class CeresTebAcceleration {
 public:
  CeresTebAcceleration(const Eigen::Matrix2d& sqrt_information)
      : sqrt_information_(sqrt_information) {}

  template <typename T>
  bool operator()(const T* const x_a, const T* const y_a, const T* const yaw_a,
                  const T* const x_b, const T* const y_b, const T* const yaw_b,
                  const T* const x_c, const T* const y_c, const T* const yaw_c,
                  const T* const time_diff1, const T* const time_diff2,
                  T* residuals_ptr) const {
    Eigen::Map<Eigen::Matrix<T, 2, 1>> residuals_map(residuals_ptr);

    Eigen::Matrix<T, 2, 1> x1(*x_a, *y_a);
    Eigen::Matrix<T, 2, 1> x2(*x_b, *y_b);
    Eigen::Matrix<T, 2, 1> x3(*x_c, *y_c);
    T dt1=*time_diff1;
    T dt2=*time_diff1;

    Eigen::Matrix<T,2,1> diff1=x2-x1;
    Eigen::Matrix<T,2,1> diff2=x3-x2;

    T dist1=diff1.norm();
    T dist2=diff2.norm();
    const T angle_diff1=ceres::normalize_theta(*yaw_b-*yaw_a);
    const T angle_diff2=ceres::normalize_theta(*yaw_c-*yaw_b);

    if (config.trajectory.exact_arc_length) // use exact arc length instead of Euclidean approximation
    {
        if (angle_diff1 != 0)
        {
            const T radius =  dist1/(T(2)*ceres::sin(angle_diff1/T(2)));
            dist1 = ceres::abs( angle_diff1 * radius ); // actual arg length!
        }
        if (angle_diff2 != 0)
        {
            const T radius =  dist2/(T(2)*ceres::sin(angle_diff2/T(2)));
            dist2 = ceres::abs( angle_diff2 * radius ); // actual arg length!
        }
    }

    T vel1=dist1/dt1;
    T vel2=dist2/dt2;

    vel1*=ceres::fast_sigmoid(T(100)*(diff1(0)*ceres::cos(*yaw_a))+diff1(1)*ceres::sin(*yaw_a));
    vel2*=ceres::fast_sigmoid(T(100)*(diff2(0)*ceres::cos(*yaw_b))+diff2(1)*ceres::sin(*yaw_b));

    const T acc_lin = (vel2-vel1)*T(2)/(dt1+dt2);


    residuals_map(0) = ceres::penaltyBoundToInterval<T>(acc_lin,T(config.robot.acc_lim_x),T(config.optim.penalty_epsilon));

    // ANGULAR ACCELERATION
    const T omega1 = angle_diff1 / dt1;
    const T omega2 = angle_diff2 / dt2;
    const T acc_rot  = (omega2 - omega1)*T(2) / ( dt1 + dt2 );

    residuals_map(1) = ceres::penaltyBoundToInterval<T>(acc_rot,T(config.robot.acc_lim_theta),T(config.optim.penalty_epsilon));

    residuals_map = sqrt_information_.template cast<T>() * residuals_map;

    return true;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  // The inverse square root of the measurement covariance matrix.
  const Eigen::Matrix2d sqrt_information_;
};


class CeresTebAccelerationStart {
 public:
  CeresTebAccelerationStart(const Eigen::Matrix2d& sqrt_information,const Twist& twist)
      : sqrt_information_(sqrt_information),_measurement(&twist) {}

  template <typename T>
  bool operator()(const T* const x_a, const T* const y_a, const T* const yaw_a,
                  const T* const x_b, const T* const y_b, const T* const yaw_b,
                  const T* const time_diff1, T* residuals_ptr) const {
    Eigen::Map<Eigen::Matrix<T, 2, 1>> residuals_map(residuals_ptr);

    Eigen::Matrix<T, 2, 1> pose1(*x_a, *y_a);
    Eigen::Matrix<T, 2, 1> pose2(*x_b, *y_b);
    T dt=*time_diff1;

    Eigen::Matrix<T,2,1> diff=pose2-pose1;

    T dist=diff.norm();
    const T angle_diff=ceres::normalize_theta(*yaw_b-*yaw_a);

    if (config.trajectory.exact_arc_length && angle_diff != 0)
    {
        const T radius =  dist/(T(2)*ceres::sin(angle_diff/T(2)));
        dist = ceres::abs( angle_diff * radius ); // actual arg length!
    }

    const T vel1=T(_measurement->linear.x());
    T vel2=dist/dt;

    vel2*=ceres::fast_sigmoid(T(100)*(diff(0)*ceres::cos(*yaw_a))+diff(1)*ceres::sin(*yaw_a));

    const T acc_lin = (vel2-vel1)/(dt);


    residuals_map(0) = ceres::penaltyBoundToInterval<T>(acc_lin,T(config.robot.acc_lim_x),T(config.optim.penalty_epsilon));

    // ANGULAR ACCELERATION
    const T omega1 = T(_measurement->angular.z());
    const T omega2 = angle_diff / dt;
    const T acc_rot  = (omega2 - omega1) / ( dt);

    residuals_map(1) = ceres::penaltyBoundToInterval<T>(acc_rot,T(config.robot.acc_lim_theta),T(config.optim.penalty_epsilon));

    residuals_map = sqrt_information_.template cast<T>() * residuals_map;

    return true;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  // The inverse square root of the measurement covariance matrix.
  const Eigen::Matrix2d sqrt_information_;
  const Twist* _measurement;
};


class CeresTebAccelerationGoal {
 public:
  CeresTebAccelerationGoal(const Eigen::Matrix2d& sqrt_information,const Twist& twist)
      : sqrt_information_(sqrt_information),_measurement(&twist) {}

  template <typename T>
  bool operator()(const T* const x_a, const T* const y_a, const T* const yaw_a,
                  const T* const x_b, const T* const y_b, const T* const yaw_b,
                  const T* const time_diff1, T* residuals_ptr) const {
    Eigen::Map<Eigen::Matrix<T, 2, 1>> residuals_map(residuals_ptr);

    Eigen::Matrix<T, 2, 1> pose1(*x_a, *y_a);
    Eigen::Matrix<T, 2, 1> pose2(*x_b, *y_b);
    T dt=*time_diff1;

    Eigen::Matrix<T,2,1> diff=pose2-pose1;

    T dist=diff.norm();
    const T angle_diff=ceres::normalize_theta(*yaw_b-*yaw_a);

    if (config.trajectory.exact_arc_length && angle_diff != 0)
    {
        const T radius =  dist/(T(2)*ceres::sin(angle_diff/T(2)));
        dist = ceres::abs( angle_diff * radius ); // actual arg length!
    }

    T vel1=dist/dt;
    const T vel2=T(_measurement->linear.x());

    vel1*=ceres::fast_sigmoid(T(100)*(diff(0)*ceres::cos(*yaw_a))+diff(1)*ceres::sin(*yaw_a));

    const T acc_lin = (vel2-vel1)/(dt);


    residuals_map(0) = ceres::penaltyBoundToInterval<T>(acc_lin,T(config.robot.acc_lim_x),T(config.optim.penalty_epsilon));

    // ANGULAR ACCELERATION
    const T omega1 = angle_diff / dt;
    const T omega2 = T(_measurement->angular.z());
    const T acc_rot  = (omega2 - omega1) / ( dt);

    residuals_map(1) = ceres::penaltyBoundToInterval<T>(acc_rot,T(config.robot.acc_lim_theta),T(config.optim.penalty_epsilon));

    residuals_map = sqrt_information_.template cast<T>() * residuals_map;

    return true;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  // The inverse square root of the measurement covariance matrix.
  const Eigen::Matrix2d sqrt_information_;
  const Twist* _measurement;
};


class CeresTebTimeOptimal {
 public:
  CeresTebTimeOptimal(const Eigen::Matrix<double,1,1>& sqrt_information)
      : sqrt_information_(sqrt_information) {}

  template <typename T>
  bool operator()(const T* const time_diff,
                  T* residuals_ptr) const {
    Eigen::Map<Eigen::Matrix<T, 1, 1>> residuals_map(residuals_ptr);

    const T timediff = *time_diff;
    residuals_map(0) = timediff;
    residuals_map = sqrt_information_.template cast<T>() * residuals_map;

    return true;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  // The inverse square root of the measurement covariance matrix.
  const Eigen::Matrix<double,1,1> sqrt_information_;
};


class CeresTebShortestPath {
 public:
  CeresTebShortestPath(const Eigen::Matrix<double,1,1>& sqrt_information)
      : sqrt_information_(sqrt_information) {}

  template <typename T>
  bool operator()(const T* const x_a, const T* const y_a, const T* const x_b,
                  const T* const y_b, T* residuals_ptr) const {
    Eigen::Map<Eigen::Matrix<T, 1, 1>> residuals_map(residuals_ptr);


    const Eigen::Matrix<T,2,1> pose1(*x_a,*y_a);
    const Eigen::Matrix<T,2,1> pose2(*x_b,*y_b);
    residuals_map(0) = (pose1-pose2).norm();
    residuals_map = sqrt_information_.template cast<T>() * residuals_map;

    return true;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  // The inverse square root of the measurement covariance matrix.
  const Eigen::Matrix<double,1,1> sqrt_information_;
};


class CeresTebKinematicsDiffDriveCeres {
 public:
  CeresTebKinematicsDiffDriveCeres(const Eigen::Matrix<double,2,2>& sqrt_information)
      : sqrt_information_(sqrt_information) {}

  template <typename T>
  bool operator()(const T* const x_a, const T* const y_a,const T* const yaw_a, const T* const x_b,
                  const T* const y_b, const T* const yaw_b,
                  T* residuals_ptr) const {
    Eigen::Map<Eigen::Matrix<T, 2, 1>> residuals_map(residuals_ptr);


    const Eigen::Matrix<T,2,1> conf1(*x_a,*y_a);
    const Eigen::Matrix<T,2,1> conf2(*x_b,*y_b);

    Eigen::Matrix<T,2,1> deltaS = conf2 - conf1;
    residuals_map(0) = ceres::abs( ( ceres::cos(*yaw_a)+ceres::cos(*yaw_b) ) * deltaS(1) - ( ceres::sin(*yaw_a)+ceres::sin(*yaw_b) ) * deltaS(0) );

    Eigen::Matrix<T,2,1> angle_vec ( ceres::cos(*yaw_a), ceres::sin(*yaw_a) );	   
    // 
    residuals_map(1) = ceres::penaltyBoundFromBelow(T(deltaS.dot(angle_vec)), T(0),T(0));

    residuals_map = sqrt_information_.template cast<T>() * residuals_map;

    return true;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  // The inverse square root of the measurement covariance matrix.
  const Eigen::Matrix<double,2,2> sqrt_information_;
};

}  // namespace ceres
