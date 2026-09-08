# 装甲板跟踪状态（tracker 输出，一条装所有）
std_msgs/Header header
geometry_msgs/Point position     # 平滑后的板心位置（相机系，m）
geometry_msgs/Vector3 velocity   # 估计速度（m/s）
float64 distance                 # 到光心距离（m）
