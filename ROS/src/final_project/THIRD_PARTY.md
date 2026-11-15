# 复用来源

- `urdf/upstream/*.xacro`：来自本地 `参考/turtlebot3_manipulation-melodic-devel/open_manipulator_with_tb3_description/urdf`，ROBOTIS Apache-2.0。仅调整包内 include 路径；原许可证在同目录 `LICENSE.ROBOTIS`。
- `config/robot.srdf`：来自同一仓库的 `open_manipulator_with_tb3_waffle_moveit/config/om_with_tb3.srdf`。保留关节组与碰撞矩阵，移除固定在 odom 的虚拟关节，使规划坐标系为移动底盘本体；每次操作重新将桌面转换到该坐标系。
- `models/delivery_marker/materials/textures/Marker0.png`：来自本地 `参考/turtlebot3_manipulation_simulations-melodic-devel/open_manipulator_with_tb3_gazebo/models/turtlebot3_rooms/ar_marker/materials/textures`，ROBOTIS Apache-2.0。
- 机器人底盘与机械臂 mesh 通过发行包 `turtlebot3_description`、`open_manipulator_description` 使用，保留各包原始许可。
- MoveIt/KDL、AMCL、GMapping、map_server、move_base/Navfn/DWA、AR Track Alvar、Gazebo 和 ros_control 使用 Melodic 发行包。
- `src/third_party/explore_lite/`：从 [hrnr/m-explore 的 lunar-devel](https://github.com/hrnr/m-explore/tree/lunar-devel/explore) 选择性复用FrontierSearch及costmap_tools，2026-10-20读取。保留BSD三条款、Bosch/Jiri Horner版权及来源说明，见该目录 `LICENSE.BSD`、`README.md`。本地修正首个边界单元的质心、points及最近距离统计，并补足直接使用的标准头文件；不修改系统包。
- 可选讯飞语音通过官方 `iFLYTEK-OP/websdk-python` 的 `IatClient` 和签名辅助接口接入；未复制 SDK 源码。对所读 SDK 默认关闭证书验证的行为，语音模块提供局部传输适配，启用证书验证。
- 模式2.2操作台使用 Qwen3-VL-4B-Instruct-GGUF 本地模型、llama.cpp server、roslibpy 和 Gradio；LEO-RobotAgent 仅借鉴观察—对话—工具—反馈的组织方式，不整体移植其 Noetic 代码。ROS1 WebSocket 接入使用 RobotWebTools rosbridge_suite 的 Melodic/ros1版本，不使用本地 `rosbridge_suite-ros2` 目录。

当前自研部分是手动action、模式接入、控制权切换、运动学校验、条件约束抓取插件，以及模式2的探索适配与地图/定位/取送顺序协调。探索节点使用上游连通搜索和边界聚类；本地补充已知空闲安全目标、规划检查、失败区分和状态输出。地图写出使用ROS map_saver，不自行替代其格式。

2026-10-20从本地 `backups/before_mode2_slam_20261019/final_project_with_autonomous_delivery.zip` 选择性恢复Delivery.action、原导航/PID交接、视觉PID对位及退离，继承当前共用完整抓放。采用顺序编排，不恢复SMACH依赖、旧文本确认或讯飞自主任务入口。GMapping、Navfn、DWA和点云显示继续复用ROS/Gazebo/RViz；复用算法和接口不作为原创算法宣称。
