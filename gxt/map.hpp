#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#include "debugstream.hpp"
#include "obstacles.h"
#include "teb_config.h"

using namespace teb_local_planner;

// 1像素代表1cm
constexpr int PIXEL_PER_METER = 400;  // 每隔1m对应的像素数
constexpr int MAP_X = 1;             // x轴显示 +- 多少米
constexpr int MAP_Y = 1;             // y轴显示 +- 多少米
//
// opencv mat 像素长宽 单位像素数 not need to change
constexpr int WIDTH = 2 * MAP_X * PIXEL_PER_METER;
constexpr int HEIGHT = 2 * MAP_Y * PIXEL_PER_METER;
//
extern TebConfig config;


/**
 * 将地图点映射到实际上
 */
inline auto MapToPoint(int x_, int y_) {
  double x=(double)(x_-static_cast<int>(WIDTH/2))/PIXEL_PER_METER;
  double y=-(double)(y_-static_cast<int>(HEIGHT/2))/PIXEL_PER_METER;
  return std::tuple{x, y};
}

/**
 * 将实际点映射到地图上
 */
inline auto PointToMap(double x_, double y_) {
  int x = (int)(x_ * PIXEL_PER_METER + static_cast<int>(WIDTH / 2));
  int y = (int)(-y_ * PIXEL_PER_METER + static_cast<int>(HEIGHT / 2));
  return std::tuple{x, y};
}

/**
 * 将路径画到地图上,注意图像坐标系不一样
 */
inline void DrawMap(cv::Mat& map, std::vector<Eigen::Vector3f>& path,
                    bool draw_arrow = true,
                    cv::Scalar line_color = cv::Scalar(0, 255, 255),
                    cv::Scalar arrow_color = cv::Scalar(255, 255, 0)) {
  for (int i = 0; i < path.size() - 1; i++) {
    // int x =
    //     (int)(path.at(i)[0] * PIXEL_PER_METER + static_cast<int>(WIDTH / 2));
    // int y =
    //     (int)(-path.at(i)[1] * PIXEL_PER_METER + static_cast<int>(HEIGHT /
    //     2));
    const auto [x, y] = PointToMap(path.at(i)[0], path.at(i)[1]);
    // int next_x = (int)(path.at(i + 1)[0] * PIXEL_PER_METER +
    //                    static_cast<int>(WIDTH / 2));
    // int next_y = (int)(-path.at(i + 1)[1] * PIXEL_PER_METER +
    //                    static_cast<int>(HEIGHT / 2));
    const auto [next_x, next_y] =
        PointToMap(path.at(i + 1)[0], path.at(i + 1)[1]);
    cv::line(map, cv::Point(x, y), cv::Point(next_x, next_y), line_color);

    // 画箭头
    if (draw_arrow == false) continue;
    auto theta = path.at(i)[2];
    const auto distance = 0.5;  // m
    const auto [end_x, end_y] =
        PointToMap(path.at(i + 1)[0] + std::cos(theta) * distance,
                 path.at(i + 1)[1] + std::sin(theta) * distance);
    // 设置箭头参数
    int thickness = 1;
    int line_type = cv::LINE_8;
    int shift = 0;
    double tip_length = 0.1;  // 箭头长度相对于线段长度的比例
    // 绘制箭头线段
    cv::arrowedLine(map, cv::Point(x, y), cv::Point(end_x, end_y), arrow_color,
                    thickness, line_type, shift, tip_length);
  }
}

/**
 * 重新初始化地图并画坐标轴线
 */
inline void ReInitialize(cv::Mat& map) {
  memset(map.data, 0, WIDTH * HEIGHT * 3);  // set map to zero
  cv::Scalar axis_color(255, 255, 255);     // 白色
  for (int i = 1; i <= WIDTH / PIXEL_PER_METER; i++) {
    cv::Point vertical_line_start(i * PIXEL_PER_METER, 0);
    cv::Point vertical_line_end(i * PIXEL_PER_METER, HEIGHT);
    cv::Point horizontal_line_start(0, i * PIXEL_PER_METER);
    cv::Point horizontal_line_end(WIDTH, i * PIXEL_PER_METER);

    cv::line(map, vertical_line_start, vertical_line_end, axis_color);
    cv::line(map, horizontal_line_start, horizontal_line_end, axis_color);
  }
  cv::Point horizontal_start(0, HEIGHT / 2);
  cv::Point horizontal_end(WIDTH, HEIGHT / 2);
  cv::Point vertical_start(WIDTH / 2, 0);
  cv::Point vertical_end(WIDTH / 2, HEIGHT);
  cv::line(map, horizontal_start, horizontal_end, cv::Scalar(0, 0, 255));
  cv::line(map, vertical_start, vertical_end, cv::Scalar(0, 0, 255));
}

/**
 * 在地图上画出障碍物
 */
inline void DrawPointObstacle(cv::Mat& map,
                              std::vector<ObstaclePtr>& obstacle) {
  gDebug(obstacle.size());
  for (const auto& obs : obstacle) {
    auto position = obs->getCentroid();
    auto [x, y] = PointToMap(position(0), position(1));
    auto r = config.obstacles.min_obstacle_dist;
    auto inflation_r = config.obstacles.inflation_dist;
    cv::circle(map, cv::Point(x, y), static_cast<int>(inflation_r * PIXEL_PER_METER),
               cv::Scalar(255, 255, 0), -1);
    cv::circle(map, cv::Point(x, y), static_cast<int>(r * PIXEL_PER_METER),
               cv::Scalar(255, 0, 0), -1);
  }
}
