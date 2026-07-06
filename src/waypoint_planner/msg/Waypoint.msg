# Waypoint.msg -- 单个任务航点
geometry_msgs/Pose pose
float64 hover_time          # 在此航点悬停的时间（秒）
float64 acceptance_radius   # 到达判定阈值（米）
