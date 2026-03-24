# pnc地图
# 双向车道，每条车道宽度相同
std_msgs/Header header                   # 消息头
float64 road_length                      # 道路长度
float64 lane_width                       # 车道宽度
visualization_msgs/Marker left_boundary  # 道路左边界
visualization_msgs/Marker right_boundary # 自车到右边线(道路右边界)
visualization_msgs/Marker midline        # 自车道左边线(道路中心线)