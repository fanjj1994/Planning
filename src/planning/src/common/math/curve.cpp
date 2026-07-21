#include "curve.h"

namespace Planning
{
  // 找匹配点下标
  int Curve::find_match_point(const Path &path, const int &last_match_point_index, const PoseStamped &target_point)
  {
    const int path_size = path.poses.size();
    if (path_size <= 1)
    {
      return path_size - 1;
    }

    double min_dis = std::numeric_limits<double>::max();
    int closest_index = -1;
    for (int i = 0; i < path_size; i++)
    {
      double dis = std::hypot(path.poses[i].pose.position.x - target_point.pose.position.x,
                              path.poses[i].pose.position.y - target_point.pose.position.y);
      if (dis < min_dis)
      {
        if (abs(last_match_point_index - i) > 100)
        {
          continue;
        }
        min_dis = dis;
        closest_index = i;
      }
    }
    return closest_index;
  }

  // 计算投影点参数（参考线）
  void Curve::cal_projection_param(Referline &refer_line)
  {
    const int path_size = refer_line.refer_line.size();
    if (path_size < 3)
    {
      RCLCPP_ERROR(rclcpp::get_logger("math"), "refer_line too short");
      return;
    }

    // 计算rs
    double rs = 0.0;
    for (int i = 0; i < path_size; i++)
    {
      if (i == 0)
      {
        rs = 0.0;
      }
      else
      {
        rs += std::hypot(
            refer_line.refer_line[i].pose.pose.position.y - refer_line.refer_line[i - 1].pose.pose.position.y,
            refer_line.refer_line[i].pose.pose.position.x - refer_line.refer_line[i - 1].pose.pose.position.x);
      }
      refer_line.refer_line[i].rs = rs;
    }

    // 计算航向角
    for (int i = 0; i < path_size; i++)
    {
      if (i < path_size - 1)
      {
        refer_line.refer_line[i].rtheta = std::atan2(
            refer_line.refer_line[i + 1].pose.pose.position.y - refer_line.refer_line[i].pose.pose.position.y,
            refer_line.refer_line[i + 1].pose.pose.position.x - refer_line.refer_line[i].pose.pose.position.x);
      }
      else
      {
        refer_line.refer_line[i].rtheta = std::atan2(
            refer_line.refer_line[i].pose.pose.position.y - refer_line.refer_line[i - 1].pose.pose.position.y,
            refer_line.refer_line[i].pose.pose.position.x - refer_line.refer_line[i - 1].pose.pose.position.x);
      }
    }

    // 计算曲率
    for (int i = 0; i < path_size; i++)
    {
      if (i < path_size - 1)
      {
        const double dis = std::hypot(
            refer_line.refer_line[i + 1].pose.pose.position.y - refer_line.refer_line[i].pose.pose.position.y,
            refer_line.refer_line[i + 1].pose.pose.position.x - refer_line.refer_line[i].pose.pose.position.x);
        if (dis <= kMathEpsilon)
        {
          refer_line.refer_line[i].rkappa = 0.0;
        }
        else
        {
          refer_line.refer_line[i].rkappa =
              (refer_line.refer_line[i + 1].rtheta - refer_line.refer_line[i].rtheta) / dis;
        }
      }
      else
      {
        const double dis = std::hypot(
            refer_line.refer_line[i].pose.pose.position.y - refer_line.refer_line[i - 1].pose.pose.position.y,
            refer_line.refer_line[i].pose.pose.position.x - refer_line.refer_line[i - 1].pose.pose.position.x);
        if (dis <= kMathEpsilon)
        {
          refer_line.refer_line[i].rkappa = 0.0;
        }
        else
        {
          refer_line.refer_line[i].rkappa =
              (refer_line.refer_line[i].rtheta - refer_line.refer_line[i - 1].rtheta) / dis;
        }
      }
    }

    // 计算曲率变化率
    for (int i = 0; i < path_size; i++)
    {
      if (i < path_size - 1)
      {
        const double dis = std::hypot(
            refer_line.refer_line[i + 1].pose.pose.position.y - refer_line.refer_line[i].pose.pose.position.y,
            refer_line.refer_line[i + 1].pose.pose.position.x - refer_line.refer_line[i].pose.pose.position.x);
        if (dis <= kMathEpsilon)
        {
          refer_line.refer_line[i].rdkappa = 0.0;
        }
        else
        {
          refer_line.refer_line[i].rdkappa =
              (refer_line.refer_line[i + 1].rkappa - refer_line.refer_line[i].rkappa) / dis;
        }
      }
      else
      {
        const double dis = std::hypot(
            refer_line.refer_line[i].pose.pose.position.y - refer_line.refer_line[i - 1].pose.pose.position.y,
            refer_line.refer_line[i].pose.pose.position.x - refer_line.refer_line[i - 1].pose.pose.position.x);
        if (dis <= kMathEpsilon)
        {
          refer_line.refer_line[i].rdkappa = 0.0;
        }
        else
        {
          refer_line.refer_line[i].rdkappa =
              (refer_line.refer_line[i].rkappa - refer_line.refer_line[i - 1].rkappa) / dis;
        }
      }
    }
  }

} // namespace Planning