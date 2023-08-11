#include <boost/smart_ptr.hpp>
#include <boost/thread/once.hpp>
#include <boost/thread/thread.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "homotopy_class_planner.h"
#include "homotopy_class_planner.hpp"
#include "inc/obstacles.h"
#include "inc/optimal_planner.h"
#include "inc/pose_se2.h"
#include "inc/robot_footprint_model.h"
#include "inc/teb_config.h"
//

#include "config.hpp"
#include "debugstream.hpp"
#include "map.hpp"

using namespace teb_local_planner;

TebConfig config;

// param
int start_theta = 0;
int end_theta = 0;

int main() {
  ReadConfigXmlFile(config);
  cv::namedWindow("path");

  // 参数配置
  PoseSE2 start(-1, 0, 0);
  PoseSE2 end(3, 4.5, 0);

  std::vector<ObstaclePtr> obst_vector;
  obst_vector.push_back(boost::make_shared<PointObstacle>(1.5, 4));
  obst_vector.push_back(boost::make_shared<PointObstacle>(1, 2));
  obst_vector.push_back(boost::make_shared<PointObstacle>(3, 6));

  ViaPointContainer via_points;
  // via_points.push_back(Eigen::Vector2d(0,5));

  RobotFootprintModelPtr robot_model =
      boost::make_shared<CircularRobotFootprint>(0.3);

  auto visual = TebVisualizationPtr(new TebVisualization(config));

  auto planner1 = new TebOptimalPlanner(config, &obst_vector, robot_model,
                                        visual, &via_points);
  auto planner2 = new HomotopyClassPlanner(config, &obst_vector, robot_model,
                                           visual, &via_points);

  cv::Mat map = cv::Mat::zeros(cv::Size(WIDTH, HEIGHT), CV_8UC3);

  cv::createTrackbar("start theta", "path", nullptr, 100,
                     [](int pos, void*) { start_theta = pos; });
  cv::createTrackbar("end theta", "path", nullptr, 100,
                     [](int pos, void*) { end_theta = pos; });
  cv::createTrackbar("weight 11", "path", nullptr, 10000, [](int pos, void*) {
    config.optim.weight_kinematics_forward_drive = pos;
  });

  if (config.gxt.show_button)
    cv::createButton(
        "ShowArrow",
        [](int sta, void*) -> void {
          config.gxt.draw_arrow = static_cast<bool>(sta);
        },
        nullptr, cv::QT_CHECKBOX, config.gxt.draw_arrow);

  while (true) {
    ReInitialize(map);
    gDebugCol1(config.optim.weight_kinematics_forward_drive);
    try {
      TIME_LOOP(main_loop);

      start.theta() = start_theta * 0.1;
      end.theta() = end_theta * 0.1;

      if (config.hcp.enable_homotopy_class_planning) {
        planner2->plan(start, end);
        std::vector<std::vector<Eigen::Vector3f>> path;
        planner2->getFullTrajectory(path);
        for (int i = 0; i < path.size(); i++) {
          DrawMap(map, path[i], config.gxt.draw_arrow);
        }
        std::vector<Eigen::Vector3f> best_path;
        planner2->bestTeb()->getFullTrajectory(best_path);
        DrawMap(map, best_path, config.gxt.draw_arrow, cv::Scalar(0,0,255));
        gDebugCol3(path.size());
      } else {
        planner1->plan(start, end);
        std::vector<Eigen::Vector3f> path;
        planner1->getFullTrajectory(path);
        DrawMap(map, path, config.gxt.draw_arrow);
      }

      DrawPointObstacle(map, obst_vector);
      cv::imshow("path", map);
    } catch (...) {
      gDebugError("ERROR gxt exception");
      break;
    }
    char key = cv::waitKey(2);
    if (key == 27 || key == 'q') break;
  }
  cv::destroyAllWindows();
  return 0;
}
