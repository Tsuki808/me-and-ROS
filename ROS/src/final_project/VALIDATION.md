# 部署与运行验证

初次部署：2026-10-16；2026-10-19 故障排查及最终修复见第 6、7 节，键盘模式扩展见第 8 节。只记录实际观察；未做多次统计试验，不据单次成功推断成功率。

## 1. 部署环境

- 本地：`F:\University\Course\dasanshang\ROS_class\ROS\src\final_project`。
- 虚拟机：`jiayuexie@192.168.162.131`，`/home/jiayuexie/ROS/src/final_project`。
- Ubuntu 18.04、ROS1 Melodic、Python 2.7、Gazebo 9；2 核、约 4 GB 内存、2 GB swap。
- 安装了导航、MoveIt、仿真、控制器、模型与 ALVAR 依赖；外层 catkin 工作区的 task1、task2、task3、final_project 统一构建成功，条件抓取 C++ 插件已实际加载运行。
- 未安装语音隔离环境，未填写讯飞凭证，未录音或调用在线识别；默认 launch 不启动语音。

## 2. 最终完整任务

从新启动的一套 GUI 仿真开始，盒子位于 A 台。执行：

```bash
source /home/jiayuexie/ROS/devel/setup.bash
roslaunch final_project simulation.launch
```

另一终端等待 READY 后执行：

```bash
source /home/jiayuexie/ROS/devel/setup.bash
rosrun final_project task.py A B
```

最终 action 输出：

```text
success: True
message: "Delivered and returned home"
elapsed_seconds: 197.723488092
```

完整经过 PREPARE、NAV_PICK、DOCK_PICK_PID、PICK、RETREAT_PICK、NAV_PLACE、DOCK_PLACE_PID、PLACE、RETREAT_PLACE、RETURN_HOME，最终 SMACH 为 `succeeded`，导航重试次数为 0。

| 指标 | 实测结果 | 含义 |
| --- | --- | --- |
| A 台视觉 PID 末次距离误差 | 4.15 mm | 相机标记到本体的对位误差，阈值 8 mm |
| A 台视觉 PID 末次方位误差 | 0.02189 rad | 阈值 0.025 rad |
| B 台 TF PID 末次距离误差 | 7.51 mm | AMCL/已知台面目标，阈值 8 mm |
| B 台 TF PID 末次方位误差 | 0.01593 rad | 阈值 0.025 rad |
| 放置验收平面误差 | 11.41 mm | Gazebo 实际物体位置，原阈值 80 mm |
| IK/FK 最大位置回代残差 | 约 1.0×10⁻¹⁶ m | 同一 URDF 模型的数学一致性，不代表实体末端精度 |
| 返回待命点实际平面误差 | 21.19 mm | 完成后独立读取 Gazebo 模型状态 |
| 返回后 AMCL 平面误差 | 26.71 mm | 与同时间戳 Gazebo 位姿对比，仅用于观测 |

完成后观察：盒子世界坐标约 `(1.19924, 0.91139, 0.20500) m`，线速度模长 `0.0053 m/s`，固定约束已释放，底盘模式为 `HOLD`，状态为 `SUCCEEDED`。任务中先检查实际抬升，松爪后检查物体位置、高度和速度，未通过写入模型位姿完成运输。

原始记录保存在本地及虚拟机包目录的 `logs/runtime/`：

- `missions.jsonl`：保留本轮开发失败、取消、续送与最终成功记录；最后一条为上述最终 A→B 结果。
- `mission_server.log`、`rosout.log`：最终仿真的阶段与运行日志。
- `final_observation.json`：完成后的模型、定位、控制与释放状态。
- `desktop_final.png`：最终桌面截图。
- `source_check.log`：最终源码汇总检查输出。

对应 ROS 日志目录为 `~/.ros/log/8b3a5c34-c929-11f1-af47-000c29cfc772/`。这些日志不作为源码发布依赖。

## 3. 其他已验证行为

本轮已通过文本入口实测非法地点拒绝、确认后提交、执行中取消并返回 `CANCELLED`；相关路由代码随后未再修改。已夹持时取消保持载物，使用 `task.py --resume B` 完成过一次续送与返回，耗时 160.29 秒、放置误差 23.86 mm。该续送运行早于最后一次定位逻辑整合，单独列示。

开发中曾出现定位偏移引起的放置失败，未改成无条件成功，也未放宽 80 mm 验收阈值。最终配置改用 `likelihood_field_prob`、500–2000 粒子、逐次重采样，并在底盘 HOLD 时围绕 AMCL 自身估计展开局部分布、更新八帧激光；未用 Gazebo 真值校正定位。激光地图与 world 由同一场景脚本生成。

## 4. 最终源码检查

只执行一次汇总检查，命令与输出保存在 `logs/runtime/source_check.log`：

```bash
source /home/jiayuexie/ROS/devel/setup.bash
python2 /home/jiayuexie/ROS/src/final_project/tools/check_project.py
```

检查范围：UTF-8/LF、Python 2/3 语法、XML 与包内启动引用、模型展开、限定指令的拒绝分支、PID 饱和抗积分累积、IK 关节限位与不可达目标。ROS 包依赖、action、控制器、TF、动态库及资源加载同时由上述真实运行验证。

结果通过，进程退出码为 0：

```text
PASS: 39 text files, Python 2/3 syntax, XML/references, command rejection, PID anti-windup and IK limits
```

上游 xacro 仍输出 `RefLink` 未加命名空间前缀的弃用提示；在目标 Melodic 环境模型已成功展开并运行，不据此声称兼容 Noetic。

## 5. 使用边界与交付状态

- 抓持采用已确认的条件约束辅助方式，不是纯摩擦夹持验证。
- 底盘使用上游默认 Gazebo world 里程计；不是纯编码器误差条件下的实体定位验证。
- 地图为已知场景生成结果，本轮未实现或验证在线 SLAM。
- B 台放置对位依赖 AMCL 与已知坐标；只有取货阶段使用随盒子移动的视觉标记。
- 反向 B→A 复用同一参数化流程，尚未单独完成反向实测。未进行长期连续任务、动态障碍或真实硬件验收。
- 讯飞在线识别仅实现源码与桥接，账号、SDK、麦克风和在线效果均未配置及实测。
- 为交付初始场景，最终测试结束后停止本轮仿真；再次启动 `simulation.launch` 会从 A 台盒子和待命点开始。只启动一套仿真，等待上次 roslaunch 及 Gazebo 退出后再重启。

## 6. 2026-10-19 导航停滞排查（参数试验未通过）

本节更新故障状态；上面的 10 月 16 日单次成功不代表当前配置可稳定完成任务。

- 原始配置已复现 `NAV_PLACE` 阶段停滞：全局路径存在，DWA 持续输出正反微速，底盘控制出口未截断这些指令。该次运行在停滞后取消，不能记为成功。
- 试验自体激光过滤后仍在载物导航阶段停滞，不能认定自体回波是唯一原因；相关过滤配置、启动项及依赖均已撤回。
- 运行中将 `forward_point_distance` 改为 `0` 能解开卡点，但从初始场景运行仍有持续绕转和倒车，未通过完整验收。
- 本地 ROBOTIS Melodic 参考配置使用 `0.325 m`。一次中途改为该值的任务完成，耗时 `283.98 s`；由于任务中修改了参数，不作为最终修复验收。
- 随后重新启动初始场景，确认启动参数为 `0.325`，保持原始激光输入，执行一次 `task.py A B`。结果为 `success: False`，`NAV_PICK: Action exceeded 100.0 wall-clock seconds`，导航重试 1 次，总耗时 `205.16 s`。全局路径存在，现场局部轨迹仅约几毫米，仍不能稳定完成任务。

上述参数试验未得到有效修复。`forward_point_distance` 已回退为原值 `0.12`，README 中未通过验收的参数修复声明已删除。未修改抓取、机械臂、PID 精对位容差和底盘速度出口的障碍停止逻辑。

本轮汇总源码检查输出保存在 `logs/debug_20261019/final_source_check.log`，结果为 PASS；后续只尝试并回退上述浮点参数，没有重复执行完整检查。最终从头运行的真实结果为失败，日志保存在同目录的 `verified_task.log`、`verified_observer.jsonl`、`verified_snapshot.log`、`verified_simulation.log`、`verified_mission.log` 和 `verified_result.jsonl`。对应 ROS 日志目录为 `~/.ros/log/91fe01e4-cb57-11f1-b869-000c29cfc772/`。

随后用户确认允许调整 DWA 与 PID 的交接方式；最终修改和实际结果见下节。

## 7. 2026-10-19 控制交接修复与完整任务

修复保留原 DWA 配置和台前精对位要求，在 `mission_server.py` 中增加近目标控制交接：

- 距导航点不超过 `0.45 m`，且 TF、激光和完整局部代价地图均未超过 1 秒时，检查到导航点的直线通道。检查用足迹外接圆扫掠，并补偿栅格单元的半对角线；实际障碍、未知单元和地图范围外均拒绝。
- 通道满足后先进入 `HOLD`，取消导航并等待 action 确认，静止更新 AMCL，再复用已有 PID 类完成低速位置和最终航向控制。过程中持续检查通道；受阻时立即停车、清除 PID 累积，在既有 40 秒超时内等待，超时后进入原有导航重试。
- 导航 PID 在 `0.04 m` 内转入最终航向控制，成功条件保持在原导航容差 `0.045 m / 0.05 rad` 内。后续视觉/TF 台前精对位仍为 `0.008 m / 0.025 rad`，抓取、放置验收和速度出口保护未放宽。
- `navigation.yaml` 启用局部代价地图完整发布，`mission.yaml` 添加交接距离，`package.xml` 显式声明已安装的 `nav_msgs` 运行依赖。未新增安装步骤、修改语音或更改 Gazebo 物理抓取插件。

开发中首次交接实现曾因一次通道拒绝立即退出，未作为成功验收；随后改为停止并有界等待，并兼容 costmap 将足迹参数回写为字符串的行为。

最终从新启动的 GUI 初始场景执行一次 `task.py A B`，全程未修改运行参数或模型位姿：

```text
success: True
message: "Delivered and returned home"
elapsed_seconds: 289.50335288
```

| 指标 | 本次结果 |
| --- | --- |
| A 台视觉精对位距离 / 方位误差 | 3.78 mm / 0.00528 rad |
| B 台 TF 精对位距离 / 方位误差 | 7.46 mm / 0.01306 rad |
| 放置验收平面误差 | 7.88 mm |
| 导航恢复次数 | 2 次，A、B 各 1 次 |
| 导航 PID 交接次数 | 5 次，含上述恢复前的交接 |
| 返航 PID 收敛时距离 / 方位误差 | 27.4 mm / 0.0009 rad |
| 完成后 Gazebo 实际返航平面误差 | 49.18 mm |
| 完成后同时间戳 AMCL 平面误差 | 28.09 mm |

完成后独立观测：盒子世界坐标约 `(1.19247, 0.90235, 0.20500) m`，线速度约 `0.00530 m/s`，`attached=false`，底盘为 `HOLD`，任务为 `SUCCEEDED`。完整经过抓取、运输、放置及返航。

本次并非零恢复运行：保守通道检查曾停车等待并触发既有重试，之后任务完成。该记录证明本次初始场景下的完整任务通过，不代表连续任务成功率、动态障碍或实体安全认证；反向任务仍未另行验收。

最终修改同步到 Windows 和虚拟机相同包目录。最终源码汇总检查通过，输出保存在 `logs/debug_20261019/handoff_source_check.log`。本次完整运行记录为该目录中的 `handoff_task.log`、`handoff_observer.jsonl`、`handoff_simulation.log`、`handoff_mission.log`、`handoff_result.jsonl`、`handoff_observation.json`。对应 ROS 日志目录：`~/.ros/log/5472f9e0-cb59-11f1-b869-000c29cfc772/`。

交付前已停止测试仿真。下次启动 `roslaunch final_project simulation.launch` 会恢复盒子位于 A 台的初始场景，再在另一终端运行 `rosrun final_project task.py A B`。

## 8. 2026-10-19 启动模式与键盘搬运

模式1为键盘驾驶并按键执行完整抓放；模式2为原有自主流程，默认值为2。模式在启动时读取，运行中不切换。两种模式共用模型、场景、机械臂规划及抓放方法；模式1的行驶、对位、退离与返航由驾驶员完成。

本轮同步 Windows 与虚拟机，`catkin_make -j2 -l2` 退出码为0，新 `Manual.action` 已生成。执行一次已有汇总源码检查，结果为 `PASS: 41 text files, Python 2/3 syntax, XML/references, command rejection, PID anti-windup and IK limits`，输出见 `logs/modes_20261019/source_check.log`。随后实际运行发现并修正键盘节点仿真时钟就绪等待、终端缓冲读取，以及 launch 模式条件的整数转换；执行权限在虚拟机恢复。这些问题的修正分别由后续键盘完整任务和模式2启动检查验证。

从初始场景启动模式1，通过 SSH 交互终端运行真实 `keyboard_control.py`，依次使用 `R/W/S/A/D/T/1/2/P/O/F/Q`。所有底盘移动均由该节点接收按键后经过速度仲裁发出，未提交 move_base 目标、调用自动对位或写入机器人/盒子位姿。操作中读取 TF/视觉提示和只读 Gazebo 状态。

| 项目 | 本次实际结果 |
| --- | --- |
| 初始模式1 | `READY MODE1`，提供手动 action，未启动文本路由 |
| 准备 | R 完成收臂与静止定位后允许键盘驾驶 |
| 驾驶 / 停车 | W/S/A/D 可驾驶；无新键后输出零速，观察到旋转速度从0.3降为0 |
| 前置拒绝 | 无新鲜标记拒绝抓取；B台定位修正后方位超限拒绝放置；手动微调后再提交 |
| 抓放锁底盘 | 观察到 `MANUAL_PLACE: running` 时为 HOLD、实际指令为零；动作完成后恢复 MANUAL |
| A台视觉抓取对位 | 距离误差1.62 mm、方位误差-0.01821 rad |
| B台TF放置对位 | 距离误差4.88 mm、方位误差0.00504 rad |
| 放置验收平面误差 | 7.19 mm，盒子稳定于B台，附着状态false |
| F完成验收 | `MANUAL_SUCCEEDED: Keyboard delivery completed and returned home` |
| 完成后AMCL返航误差 | 约45.1 mm、0.0168 rad，满足原0.09 m / 0.1 rad条件 |
| 完成后Gazebo只读返航误差 | 约46.8 mm、0.0174 rad |
| 导航重试 / 导航PID交接 | 0 / 0 |
| 总历时 | 1025.45秒，包含SSH观察、手动调向及等待时间 |

该次为单次键盘完整A→B→home运行，最终任务记录含 `operation_mode=1`、`outcome=succeeded`。运行中曾斜向接近台面而受碰撞限制，经手动退离并改为正面接近；没有绕过碰撞或放宽8 mm / 0.025 rad对位条件。首次抓取存在标记短时过期拒绝，静止后重新提交成功。

原始记录保存在 `logs/modes_20261019/`：`mode1_mission.log`、`mode1_simulation.log`、`mode1_result.jsonl`、`mode1_deadman_observation.json`、`mode1_pick_observation.json`、`mode1_place_observation.json`、`mode1_finish_observation.json`。ROS日志目录为 `~/.ros/log/dc3ff898-cb5c-11f1-b869-000c29cfc772/`。部分只读短进程退出时出现Melodic rospy线程清理异常，观测内容及任务进程继续正常运行；未据此修改系统ROS包。

取消后保持载物及R恢复路径已实现，本轮未在载物臂动作中逐项实测所有取消时序。反向任务、连续重复任务及真实硬件仍未验收。讯飞环境保持未配置。

最后启动模式2做接口隔离检查，退出码为0：自主 delivery action 可连接、手动 action 不存在、文本路由正常；请求 MANUAL 来源并连续发布手动速度时，观察到的21帧底盘输出均为零。结果保存在 `mode2_isolation_check.log`，启动日志为 `mode2_startup.log`。本轮未重复完整自主任务，先前的完整自主运行记录见第7节。

交付前停止了键盘节点及测试仿真，等待 Gazebo 进程退出。下次选择 `mode:=1` 或 `mode:=2` 启动，即从 A 台盒子和待命点恢复初始场景。

## 9. 2026-10-19 全模式传感器与8×6 m迷宫

本节对应新迷宫。第2、7、8节的完整搬运成功均发生在旧场景，不能用作新场景模式2自主搬运已通过的证据。本轮不提交模式2自主任务，不改其状态机、动作、DWA/PID、抓放或文本/语音路由。

修改集中于机器人生成、场景/地图生成、共用launch、RViz、Gazebo插件依赖及说明。Windows和虚拟机各保留修改前源码，虚拟机通过直接逐文件内容对照确认上述冻结文件保持原样（`logs/maze_20261019/maze_scope_check.log`）。A/B台、物体、home、俯视抓取相机参数保持原样。

首次汇总检查在模型展开阶段发现：机械臂专用Waffle xacro实际引用Pi普通相机的Gazebo定义，不能仅修改原插件参数即得到深度。已在本包生成阶段明确改为depth传感器及OpenNI插件，保留R200几何与既有光学坐标系，不改系统包。该失败输出保留为`initial_source_check.log`。修正后完成一次最终汇总检查及一次`catkin_make -j2 -l2`，退出码均为0：

```text
PASS: 41 text files, Python 2/3 syntax, XML/references, RGB-D/RViz interfaces, world/map geometry, command rejection, PID anti-windup and IK limits
```

检查覆盖UTF-8/LF、XML/YAML、Python语法、URDF展开、接口/坐标系、插件文件、同名光学link唯一性及RViz启用项；按World静态碰撞几何逐格核对360×280地图全部100800个单元。9个矩形（外墙、隔墙、台面）与4个圆柱的视觉/碰撞几何一致。原上游`RefLink`的Melodic弃用提示、CMake的旧版策略提示仍存在，未修改上游资源消除提示。

模式1实际启动后，订阅得到`/odom`、`/scan`、普通相机、全部Kinect RGB/深度/标定/点云话题。初始观测窗口约129秒：里程计约19.90 Hz（墙钟）/28.04 Hz（仿真时间），激光约3.53/4.97 Hz；图像约1.49/2.07 Hz。RGB-D为640×480，普通相机为1280×960；深度编码32FC1，抽样有效距离约0.402–4.085 m。已变换的点云中，中央墙南表面约y=-0.400 m、北外墙内表面约y=2.950 m，与World一致。以上为单个窗口，不是全程最低帧率或性能保证。

实际RViz显示发现默认Odometry协方差图形遮挡主视图。读取`/odom`确认Z/横滚/俯仰方差为1e12后，仅关闭协方差绘图，保留方向箭头及原消息；随后观察到地图、机器人、红色激光、里程计箭头、彩色点云，以及普通/RGB/深度三个图像面板同时显示，Global Status为Ok。截图约7 FPS，低于目标15 FPS；分辨率与更新率配置未降低。截图见`rviz_no_covariance.png`，场景截图见`mode1_desktop.png`。

模式1在同一套新场景中完成A→B→home：底盘只经过原`keyboard_control.py`与速度仲裁，未提交move_base目标、调用自主PID或设置模型位姿。先用SSH交互终端驾驶至A区，之后临时验收脚本通过PTY向该键盘节点注入按键，根据TF/视觉反馈选择W/S/A/D/T及P/O/F；这是键盘接口的脚本辅助验收，不是全程人工手按演示，也未加入产品运行逻辑。临时脚本启动曾误用Python3、未兼容ALVAR空pose header；修正为Python2并复用键盘节点的header处理后继续。旧的驱动进程结束存在延迟，相关日志保留，不能解释为多次独立完整任务。

最终返回`MANUAL_SUCCEEDED: Keyboard delivery completed and returned home`，任务记录`operation_mode=1`、`outcome=succeeded`。主要经由(±0.7,-1.2)南侧通道换侧，保持实体墙柱碰撞。实际任务进程记录1次抓取因视觉过期拒绝、9次放置因静止AMCL更新后误差超限拒绝；继续键盘微调或静止重试后完成，未放宽阈值。新迷宫中该流程有明显的定位敏感性，不能宣称稳定成功率。

| 指标 | 本次结果 |
| --- | --- |
| A视觉抓取对位距离 / 方位误差 | 0.638 mm / -0.00899 rad |
| B台TF放置对位距离 / 方位误差 | -3.579 mm / -0.00487 rad |
| 放置平面误差（既有验收） | 23.05 mm |
| 导航重试 / 导航PID交接 | 0 / 0（键盘模式） |
| IK/FK最大回代残差 | 6.21×10⁻¹⁷ m，仅为模型数学一致性 |
| 总历时 | 1489.03秒，包含观察、驾驶、重试及等待 |

完成后只读观测：盒子约(1.22290,0.90264,0.20498) m，速度为零、attached=false；AMCL返航平面误差约47.1 mm，航向误差约-0.00563 rad。原始结果见`maze_mode1_result.jsonl`和`maze_mode1_finish.json`；完整按键终端、任务驱动、传感器观测和运行日志均在同一目录。模式1 ROS日志目录为`~/.ros/log/3aa9e30a-cb65-11f1-b869-000c29cfc772/`。

运行中另检查CameraInfo发现继承的`hackBaseline=0.07`使RGB投影矩阵出现双目偏移。已仅将本包RGB-D生成参数改为0，保持RGB和深度同一光学系；这一修正及Odometry协方差显示修正由随后模式2真实启动检查验证，不重复整个模式1搬运。其余传感器配置与本次完整搬运相同。

模式2重新从初始场景启动，默认启用RViz，启动检查退出码为0；`/final_project/delivery`可连接、`command_router`运行、未启动手动action。九个传感器/标定话题均收到消息，RGB/深度CameraInfo投影偏移为0，光学Z轴在底盘坐标中约为(0.999999,0.000796,0.000796)，与前向一致，未重复增加TF。模式2深度抽样有效范围约0.402–4.083 m。RViz截图显示三个图像、地图、机器人、激光与点云，Global Status为Ok，截图约10 FPS；见`mode2_rviz.png`和`maze_mode2_startup_check.log`。原自主逻辑保持不变，不提交自主任务，不调整导航参数，新迷宫自主搬运仍待后续适配。模式2 ROS日志目录为`~/.ros/log/5b32e4b0-cb69-11f1-b869-000c29cfc772/`。

后续显示与标定修正只做针对性检查：最终RGB-D生成参数和CameraInfo由实际模式2启动验证，RViz由最终配置启动验证；不重复整套构建、源码检查或搬运。最后对本轮交付文件检查UTF-8/LF、Python语法和配置可解析性，结果见`delivery_files_check.log`。临时验收脚本置于日志目录，`keyboard_demo.py`为现场最后续行阶段的副本，不是从初始场景自动复现完整键盘流程的产品入口。启动仍按README使用原键盘节点。

本轮结束已停止两套测试仿真及临时观测/键盘进程，源码、World、地图、RViz、文档与真实日志同步到本地和虚拟机。未安装新依赖（所需OpenNI插件已存在），未配置讯飞。清晰档受当前2核/4GB虚拟机性能限制；10 Hz传感器与15 FPS显示是配置目标，实测未全部达到，不静默降低配置。

## 10. 2026-10-19 模式3语音控制

本轮新增模式3作为模式1的语音版本。现有手动action和速度来源增加模式3接入，默认模式2、模式1键盘、模式2自主逻辑和抓放条件保留。新指令要求“小车”前缀，方向最长连续10秒，微调为原细调速度点动0.25秒。语音动作不自动导航或对位。

修改前源码备份：本地工作区 `logs/before_voice_mode3_20261019.zip`，虚拟机 `/tmp/before_voice_mode3_20261019.tar.gz`。冻结文件通过直接内容比较，不进行哈希验证。

虚拟机已安装独立Python 3.8和 `/home/jiayuexie/ROS/.venv-voice`；讯飞SDK为xfyunsdkspeech 0.0.4、xfyunsdkcore 0.0.3，保留ROS Python 2。已按授权写入虚拟机用户私有听写配置，权限600，不在源码或日志记录凭证。后续用户配合发声，已验证Windows麦克风经VMware透传、连续在线识别及下述短控制流程。

执行结果保存于 `logs/voice_20261019/`：

| 项目 | 实际结果 |
| --- | --- |
| 汇总静态检查 | 通过：43个文本文件的UTF-8/LF、Python 2/3语法、XML/引用、语音白名单拒绝；原有传感器、地图几何及运动学检查亦通过，见 `source_check.log` |
| 修改范围 | 35个原文件直接逐字节比较相同；修改的8个原文件为说明、构建、launch、手动模式接入、检查及语音入口，见 `scope_check.log` |
| 构建 | `catkin_make -j2 -l2`退出码0；保留原xacro RefLink与CMake策略提示，见 `build.log` |
| 模式1、模式2 | 两者实际启动检查退出码0，原对应action、路由隔离和RViz进程均正常；均收到里程计、激光、普通图像、RGB、深度与点云；未发送自主任务，见 `mode1_check.log`、`mode2_check.log` |
| 模式3文本控制 | 退出码0：前缀/否定/复合语句拒绝、准备时锁车、原prepare动作、微调、方向替换、10秒停车、取消和退出，见 `mode3_check.log` |
| 10秒截止 | 右转最后一个非零手动速度样本位于发令后9.996秒；状态查询及非法复合指令未续期 |
| 微调 | 发布0.02 m/s，20 Hz采样非零首尾跨度0.200秒，随后为零；配置截止0.25秒，以上不是实际位移保证 |
| 语音进程异常停车 | 使用明确的 `/bin/false` 子进程替身检查监督路径，控制节点退出码1且末条速度为零；不当作真实录音故障测试 |
| 讯飞鉴权及在线样本 | 参考仓库 `xfyun-webapi-master/bin/audio/iat_pcm_16k.pcm` 经真实在线听写，最终文本为“语音听写可以将语音转为文字”；没有向机器人发指令，见 `speech_sample_result.json` |
| 真实麦克风与连续识别 | 通过：单句“小车状态”取得完整最终结果，25秒观察中进程存活并正常轮换；见 `continuous_microphone_result.json` |
| 真实语音ROS短流程 | 通过：“状态→准备→微调前进→停止”均被接受，收臂完成后允许驾驶；见 `real_voice_events.jsonl`、`real_voice_result.json` |
| 真实语音微调输出 | 原仲裁输出0.02 m/s，首次非零至归零约0.251秒；最终文本到达至首次底盘非零输出约0.081秒，均为观察进程墙钟时间，不是说话结束至执行的识别延迟 |
| 完整语音搬运与运动中语音停车 | **未验证**：用户要求保存结果并停止，完整A→B→home留待后续；本次“停止”到达时微调已自动归零 |

必要检查中出现并修复两处执行问题：首次检查包装脚本在source ROS前启用了 `set -u`，由ROS初始化读取未设置变量导致提前退出；调整为source后启用。首次启动时Windows打包未保留原脚本执行位，任务节点未启动，已从修改前tar恢复原权限；最终源码归档从虚拟机生成以保留权限。首次模式3观察脚本未解码Python 2收到的UTF-8状态，报UnicodeDecodeError，已显式解码；只补跑未通过的模式3检查，未重复模式1/2或构建。相应失败日志保留为 `initial_mode1_*`、`initial_mode3_*`。

后续真实麦克风验收先录制10秒PCM（320000字节），最初输入较弱，RMS约-66.10 dBFS，识别为“小车主小车状态”，未向ROS发令。随后将PulseAudio实际输入 `alsa_input.pci-0000_02_02.0.analog-stereo` 从42%调至100%，不使用输出monitor。第一轮连续识别得到“小车状态小车状态”，用户确认只在开头连续重复；该复合结果不符合单条白名单，未放宽解析。第二轮仅说一次后，完整识别“小车状态”。记录中4.599秒从进程启动计时，不能作为识别延迟。录音与独立连续检查均不发布ROS控制指令，数值和文本分别保存在 `microphone_result.json`、`continuous_microphone_initial_result.json`、`continuous_microphone_result.json`。

保存记录时更正了临时脚本的两处元数据：首录音的“识别成功”由子串包含改为完整匹配（该条为false，另保留包含结果true）；独立连续检查标注为“不发布ROS指令”，不再误写“没有ROS机器人运行”。原音频数值和识别文本不变，没有重新录制。

集成验收发现用户已运行的模式3语音节点在首次监听提示时退出。用户关闭原仿真并确认后，从初始场景重新启动，复现故障。临时诊断定位到ROS Melodic Python 2的 `rosgraph.roslogging.RosStreamHandler.emit`：中文日志的Unicode消息在 `str(record_message)` 触发UnicodeEncodeError；只编码消息后，Unicode格式串及Unicode节点名仍分别导致UnicodeDecodeError。最终仅修改 `scripts/voice_control.py` 两行：日志直接传入UTF-8字节串，`init_node`使用ASCII字节串节点名，避免日志处理器混合类型。没有修改ROS系统包、指令解析、控制参数、抓放或导航逻辑。

补修使用实际 `rosrun final_project voice_control.py` 入口接入本次模式3仿真，真实监听、中文反馈和四条指令正常。原 `prepare` 返回 `MANUAL_SUCCEEDED: Arm stowed; voice driving enabled`，准备期间保持HOLD，成功后进入MANUAL。随后真实“小车微调前进”使 `/final_project/manual_cmd_vel` 和 `/om_with_tb3/cmd_vel` 依次出现0.02 m/s并归零；真实“小车停止”被独立接受。此阶段没有文本注入、自动导航或位姿写入；背景讲话12条按前缀规则拒绝。原汇总检查和构建没有重复，本次两行修复由实际Python 2节点与真人指令验证。临时诊断脚本仅位于日志目录，不接入产品启动。

按用户最新要求，已停止本次控制节点、录音子进程、观察进程和测试仿真，进程检查未发现对应roslaunch、Gazebo、RViz或arecord继续运行。用户原仿真先由用户主动关闭，本次仅停止自己启动的进程。新旧日志及源码同步本地与虚拟机；完整语音搬运、运动中的语音停止及实际说话结束至执行的延迟仍待后续。模式2新迷宫自主搬运仍未适配或验收，旧场景成功记录不作为模式3完整搬运通过的证据。

## 11. 2026-10-19 模式2改为实时SLAM与点击导航

本节为当前主线结果，取代前文模式2自主搬运的启动与接口说明。按用户确认，采用激光GMapping实时二维建图与定位、RViz目标导航，继续显示现有Kinect点云；不实现点云融合三维SLAM或自动探索。

修改前已在工作区src之外完整备份：本地 `ROS/backups/before_mode2_slam_20261019/final_project_with_autonomous_delivery.zip`，虚拟机同目录的 `final_project_with_autonomous_delivery.tar.gz`。本地另保存该tar归档以保留Linux执行权限，以及修改前课程README。完整包备份含历史日志，不含包外讯飞凭证或语音venv。

主线删除原Delivery.action、自主搬运SMACH状态机、搬运专属导航/PID交接、自动对位/退离、task.py、command_router.py、send_text.py和自动搬运验收入口。保留Manual.action及模式1/3完整抓放、IK/FK、8 mm / 0.025 rad对位校验。直接源文本比较确认16个共用手动/机械臂方法逐字不变，9个控制/模型/World/地图/配置文件内容不变，见 `logs/slam_20261019/preserved_scope.log`。原navigation.yaml未修改，模式2单独覆盖通用导航终点容差0.10 m / 0.15 rad。

虚拟机已安装 `ros-melodic-gmapping` 1.4.1及 `ros-melodic-openslam-gmapping` 0.2.1。仅模式2启动GMapping，不启动固定地图、AMCL、mission_server或旧任务路由。模式1/3继续使用原固定地图和AMCL；本轮未重新执行两者完整搬运或真实录音。

一次汇总源码检查及一次 `catkin_make -j2 -l2` 均通过，见 `source_check.log`、`build.log`：

```text
PASS: 41 text files, Python 2/3 syntax, XML/references, RGB-D/RViz interfaces, world/map geometry, voice rejection, SLAM launch isolation and IK limits
```

首次模式2启动，机械臂实际已收好，但新节点读取了仅含轮关节的 `/joint_states`，错误判定收臂未完成并保持HOLD。已按原节点方式将输入映射到 `/final_project/joint_states`，并保留执行结束后最多2秒的反馈等待；随后仅补启动该节点，确认 `READY MODE2` 和NAV，不重复全套构建。最终launch已包含同一映射。失败日志 `mode2_simulation.log` 保留。

同一套模式2仿真中执行一次导航观测。收到里程计、激光、普通图像、Kinect RGB、深度与点云六类消息，RViz进程存在；点云坐标系为 `camera_rgb_optical_frame`。目标通过与RViz 2D Nav Goal相同的 `/move_base_simple/goal` 接口发布，未实际操作鼠标，也未直接写入底盘速度或机器人位姿。

| 项目 | 真实结果 |
| --- | --- |
| 目标1 `(0.8, 0.3)`、朝向π/2 | SUCCEEDED；墙钟约23.03秒；观测位置误差约86.3 mm |
| 目标2 `(-0.8, 0.3)`、朝向π/2 | 最初180秒墙钟观察窗口超时；其后原导航继续执行并返回SUCCEEDED，未重新发目标 |
| 第二段绕障 | 轨迹从中央墙南侧绕行；穿越中央区域的样本均位于y<-0.4 m；窗口内记录路径约4.19 m |
| 第二目标到达日志 | 目标开始仿真时间123.544秒，Goal reached为262.672秒，相差139.128仿真秒；未完整记录最终墙钟历时 |
| 后续只读观测 | 导航状态为3、Goal reached；位置约(-0.71361, 0.24955)，距目标约100.04 mm；底盘指令为零 |
| 地图变化 | 已知栅格由15370增至51551，原观察中收到112次地图消息；最终地图分辨率0.025 m |

第二目标的终点收敛偏慢，不能把整次有界观测写为通过。`navigation_result.json` 原样保留 `passed=false` 和 `Navigation wall-clock timeout`；`navigation_followup.json` 单独记录后续成功、轨迹及地图证据。后续位置是到达后的采样，不作为精确到达瞬间误差。诊断时曾在目标已经到达后临时将运行中的forward_point_distance改为0，随后恢复0.12；该诊断没有促成到达，源码参数始终保持0.12。未据此修改DWA参数或重复路线。

停止测试后，清理本包devel中残留的Delivery生成文件及各语言索引，只执行 `final_project_generate_messages` 目标重新生成Manual接口；Python可导入ManualAction、不再导出Delivery，见 `generated_messages.log`。未清空整个build/devel，未重复完整构建。

静态墙体绕行、实时地图扩展、传感器消息和目标接口已有实际证据。真实鼠标操作、临时新增动态障碍、不可达点失败路径、连续多目标成功率和新主线模式1/3完整搬运本轮未另行验证；RViz各显示项沿用原配置并增加局部代价地图及目标箭头，本轮未另存屏幕截图。点云是实时显示，不能据此宣称完成三维SLAM。原有上游xacro/CMake提示保留。

已仅向本次启动的控制节点和仿真进程组发送SIGINT，确认测试Gazebo及RViz退出。源码、文档与本轮记录同步本地和虚拟机。下次使用 `roslaunch final_project simulation.launch mode:=2` 从初始场景重新建图，等待READY MODE2后在RViz设置可达空闲地面的目标。

## 12. 2026-10-20 全自动建图与自主取送（Windows源码）

本节取代第11节作为当前主线说明；前文点击导航、旧场景搬运和语音结果均为历史记录。**本轮仅修改Windows本地代码，未连接或部署虚拟机，未执行Melodic构建或仿真。不能据此前的成功记录推断本轮自动流程已运行成功。**

改造前完整包已备份到 `ROS/backups/before_mode2_auto_mapping_20261020/final_project_before_auto_mapping.zip`，共182个文件，含历史日志；同目录保存课程README与备份说明。保留 `before_mode2_slam_20261019` 的原自主搬运备份。

实现范围：

- 模式2协调节点自动顺序执行收臂、边界探索、保存地图、互斥切换AMCL、一次A→B→home取送，不恢复文本/语音确认路由或SMACH。
- 包内保存explore_lite的BSD边界搜索源码，编译为独立探索可执行文件；仅输入实时SLAM地图、激光、TF和规划服务，不读取World障碍布局或预制地图，不用固定巡逻点。
- 1 Hz评估、0.30 m有效边界、安全空闲目标与可达性检查；无进展60秒仿真时间清图后重试一次。明确区分EXHAUSTED、RETRYING、BLOCKED和WAITING；未解决失败阻止进入搬运。总探索上限1200秒仿真时间，超时停车。
- 完成判定要求观测有效、边界耗尽、无未解决失败，以及固定场景比较区域内连续10秒仿真时间的分类变化≤0.1%。按地图坐标直接比较，记录窗口内曾变化的单元，避免变化后又恢复被忽略。
- 停车后冻结地图，map_saver保存PGM/YAML，直接核对分辨率、原点、尺寸及纵向翻转后的像素。保存停止时SLAM位姿，停止探索/GMapping/建图move_base，确认退出后再启动AMCL和最终地图导航。
- 用停止位姿初始化AMCL，等待新鲜定位、TF及八次静止更新；协方差与交接偏差合格，并能规划到A/B接近点及home，才启动Delivery服务端。场景点通过启动时scene→map关系转换。
- 从旧备份恢复导航、通道检查约束下的PID交接、视觉对位和退离；复用当前完整抓放、IK/FK、8 mm / 0.025 rad及条件约束抓取。模式1/3仍仅接受MANUAL，模式2依阶段使用NAV/DOCK。
- `/final_project/mode2/cancel`为Trigger取消入口；子进程启动/停止与阶段切换在主线程执行。失败、取消或成功均停车；已夹物品不自动松爪，不重复提交任务。

本机检查入口为 `tools/check_project.py --static-only`，使用Windows Python 3执行，原ROS环境检查入口保留。2026-10-20执行一次汇总检查，初轮日志为 `logs/auto_mapping_20261020/source_check.log`：在launch引用检查处，将只存在于虚拟机的 `../../.venv-voice/bin/python` 误判为本机缺失文件而中断。已把该精确路径明确列为环境依赖跳过项，没有放宽包内文件检查。

随后从 `launch/simulation.launch` 中断位置继续，已完成的文件检查不重复；余下编码/配置检查、课程README检查和全部无ROS断言执行通过，但临时续查脚本替换os.walk后未兼容TemporaryDirectory清理所用的关键字参数，在清理阶段报TypeError，见 `source_check_followup.log`。该错误发生在地图像素断言之后，未发生产品逻辑断言失败。未重新执行整套检查，仅将尚未到达的直接比较部分单独运行，退出码0，见 `scope_check.log`。保留两次检查包装错误，不将其写为一次无错误的完整通过日志。

| 检查项 | 本机结果 |
| --- | --- |
| UTF-8/LF、XML/YAML、Python语法及兼容写法规则 | 已完成；这里只是Python 3解析及静态规则，不是Python 2实际执行 |
| launch、CMake路径、依赖、Manual/Delivery Action及模式隔离 | 已完成；VM语音解释器路径明确跳过，模式1/3默认导航参数保留 |
| 完成与受阻、地图平移/旋转/原点变化、10秒稳定窗口及累计变化 | 无ROS断言通过 |
| 冻结地图PGM纵向翻转、未知/占据/空闲分类及损坏拒绝 | 无ROS断言通过 |
| 取消后禁止阶段推进与子进程启动、原PID抗饱和 | 无ROS断言通过 |
| 与改造前备份直接内容比较 | 18个共用方法、clip/horizontal_ik及18个冻结文件不变；包括原完整抓放、键盘、语音、模型生成、传感器、World、地图及URDF |

实现期间的人工源码审阅补齐：catkin脚本入口下直接导入原Mission源码；导航取消确认后才请求下一条路径；模式2规划服务不清除起点周围代价；扫描必须包含有效距离且有扫描坐标TF。以上未替代实际ROS/C++运行检查。

尚未验证：Python 2实际运行、ROS模块导入、xacro/URDF展开、Gazebo插件加载、C++探索/catkin构建，以及任何新模式2仿真结果。Windows解析与断言通过也不代表上述运行项目通过。

后续虚拟机验收：

1. 另行同步本轮源码，按项目环境脚本安装已声明的Melodic依赖并构建，加载工作区。只运行一套仿真，执行 `roslaunch final_project simulation.launch mode:=2`，不发人工目标、方向、确认或搬运任务。
2. 观察状态、探索目标和边界，确认小车主动扩展地图并绕过墙柱；四项传感器及点云继续发布。人工点击目标不应覆盖自动任务。
3. 完成后检查本轮 `~/.ros/final_project/maps/<运行时间>/` 的PGM、YAML和run.json，确认完成条件及地图路径；观察GMapping与AMCL从不同时发布map→odom，定位就绪前底盘HOLD。
4. 在同一次运行中核对A抓取、退离、B放置及home返航，记录原对位/IK/FK/放稳条件和最终停车；不以单次建图或单次抓放替代完整流程验收。
5. 针对性检查取消、传感器丢失、导航受阻、保存或交接失败时停车且不推进阶段；持物时不自动解除约束。验证结束关闭本次测试仿真。失败记录保留，不通过放宽容差获得成功。

本轮不改讯飞环境，不录音，也不更改模式1/3原操作方式。探索效率、最终地图质量、AMCL收敛及新迷宫完整取送均需上述真实运行证据。

## 2026-10-20：模式2持续YOLO观察窗口

模式2默认启动独立 `yolo_preview.py` 与已安装的 `image_view`，显示 `/final_project/vision/image_annotated`。复用原模型及掩码HSV判色，叠加框、类别、颜色、置信度、目标颜色及推理耗时。观察节点只发布图像，A台原SELECT_BOX推理与抓取决策保持原样；模式1/3不启动观察节点。`yolo_view:=false` 关闭窗口及额外推理。

修改前分别备份本地与虚拟机完整包至各自 `ROS/backups/before_mode2_yolo_view_20261020_1510/`。本地一次针对性检查通过UTF-8/LF、Python语法兼容规则、XML及文档围栏；与备份直接内容比较，70个原有文件保持一致，包括探索、地图交接、任务识别、抓放、传感器、场景及模式1/3控制文件。

虚拟机检查：实际Python 2语法、人工构造图像上的标注绘制、模式2启动与依赖接入检查通过，`catkin_make -j2 -l2`完成；ROS节点查找接口已找到新增可执行脚本。构建与检查日志为 `/home/jiayuexie/ROS/logs/mode2_yolo_view_build.log`，检查入口为 `tools/check_yolo_preview.py`，不启动ROS主节点、相机、模型或GUI。

本次仅同步源码、说明并完成上述检查，未运行模型推理、打开窗口或启动新仿真；检测到用户已有仿真运行，未停止或重启。新增窗口在关闭现有仿真后、下一次从虚拟机图形桌面终端启动模式2时生效。实际窗口、连续识别帧率及额外CPU负载对任务的影响由用户自行验收；上述结果不代表真实识别或完整搬运通过。

## 2026-10-22：模式2探索停车修复与虚拟机复验

本节为本轮调试记录。用户授权停止失败仿真并重新验证；另确认仅建图阶段禁止倒车、启用原生振荡检测和转向恢复、增加边界几何跨度判据，以及统一0.30 m安全包络。修改前完整包与课程README分别保存在本地和虚拟机 `ROS/backups/before_mode2_debug_20261022/`，原BLOCKED记录和ROS日志保留在 `ROS/logs/mode2_debug_20261022/`。

本轮最小修改：

- 保留原始 `/scan`，仅供GMapping的扫描将正无穷无回波转换为 `range_max`；GMapping在3.45 m有效范围内清空空闲射线，不将3.5 m端点标为障碍。
- 协调器和探索器在现有新鲜度检查内最多等0.1秒扫描时刻TF，避免扫描先于对应里程计TF到达时反复HOLD。
- 建图阶段DWA只前进和转向，启用15秒/0.10 m振荡检测及带碰撞检查的原生转向恢复；保留原前后激光停车保护。
- 有效边界除原0.30 m累计长度外，其点间最大跨度也须至少0.30 m；失败不再对应任何有效边界时解除，不将仍然有效的受阻边界当作探索完成。
- 建图全局/局部代价地图使用覆盖0.30 m圆的16边形外切包络，与探索目标的转向间距一致。AMCL搬运阶段实际恢复原矩形、倒车和恢复参数。
- 搬运节点等待首个有效仿真时钟后再初始化动作客户端，避免晚启动时从零跳到当前时刻导致立即超时。不改共享机械臂方法、容差、模型、场景、传感器、语音及模式1/3操作逻辑。

### 已观察结果与保留失败

`recovery_mapping` 在仿真533.614秒报告BLOCKED、未解决失败2个，未保存地图。残留边界实际最大跨度约0.177 m和0.280 m，原累计长度却为0.35 m和0.45 m；据此增加用户确认的几何判据。

`span_mapping` 观测到底盘中心距地图障碍约0.253 m，小于矩形底盘转向包络约0.30 m，转向恢复受碰撞限制；随后用户确认仅建图使用统一安全包络。

`envelope_mapping` 无人工目标完成探索：仿真610.756秒EXHAUSTED、边界0、失败0；连续稳定后621.834秒HOLD，622.910秒保存地图，624.376秒进入定位，625.686秒观察到AMCL的map→odom，627.526秒进入DELIVERING。保存地图逐格/元数据验证、八次静止AMCL更新及A/B/home路径检查均通过。GMapping节点及原move_base先退出，再启动AMCL；搬运阶段参数现场确认为矩形footprint、min_vel_x=-0.06、oscillation_timeout=0、clearing_rotation_allowed=false。Gazebo、RViz和YOLO窗口均存在，传感器与标注图像持续发布。

该轮搬运节点启动后约0.16秒错误报告90秒导航连接超时，尚未提交搬运。只读探针复现：节点时钟初值0，首次3秒动作连接等待在约0.032秒内返回false，此时时钟已跳至857.154秒；时钟就绪后的第二次等待立即成功。修复后复用该轮保存地图进行独立搬运检查，实际节点通过启动并进入PREPARE、NAV_PICK及0.449 m处的PID交接。**这是分段检查，不是同一次完整运行成功。**

分段检查随后被Gazebo服务端段错误中断：仿真时钟停在955.628秒，退出码139；apport转储和GDB回溯指向 `gazebo::physics::World::Update()` 的 `worldUpdateBegin` 回调分发处。未据此修改Gazebo系统库或抓放逻辑，触发原因尚未完全确定。原报告、完整回溯及分段结果保留在 `envelope_mapping/gzserver.crash`、`crash_backtrace.txt`、`targeted_delivery_result.json`。

### 最终复验

包含仿真时钟修复的 `clock_mapping` 从头复验仍在探索中遇到转向栅格碰撞，未到地图保存。只读检查在0.05 m局部栅格上复现：含默认padding的包络外半径约0.320 m，距最近致命栅格中心约0.336 m，部分转向角的包络边缘穿入致命格。用户确认后仅建图将探索安全半径和16边形包络统一扩大至0.35 m。

`margin_mapping` 的0.35 m复验仍出现转向碰撞和路径失败；实际包络外半径约0.371 m，而膨胀半径仍为0.35 m，缺少扩大包络外的代价梯度。按用户再次确认，仅建图将全局/局部膨胀半径改为0.50 m、衰减系数仍为8.0，局部栅格改为0.025 m。搬运继续通过原配置恢复0.35 m膨胀半径和0.05 m局部栅格。两轮中断记录均保留，不记作通过。

上述配置同步后从初始场景启动 `consistent_mapping`，实际参数为安全半径0.35 m、膨胀半径0.50 m、局部栅格0.025 m。仿真732.554秒仍报告BLOCKED、1个未解决失败，未保存地图；剩余边界相连未知区实测117格、0.073125 m²。该轮失败记录已保留并停止，不能记作完整取送通过。

本轮一次汇总静态检查通过，日志 `ROS/logs/mode2_debug_20261022/source_check.log`：66个文本文件UTF-8/LF、Python语法/兼容规则、XML/YAML/路径/构建声明；完成、失败、取消、地图对齐、稳定窗口、地图像素核验和时钟零值跳变断言；直接内容比较确认18个共享算法、clip/horizontal_ik、14个冻结文件及原场景不变（此前已授权的pick/place模型身份参数化与三个彩盒除外）。Windows检查本身不证明ROS运行通过；本轮C++修改已另在虚拟机 `catkin_make -j2 -l2` 构建通过，见 `span_build.log`，后续仅修改已在实际启动中加载的Python与launch/config，没有重复无关构建。

### 用户确认的小残区容许策略

用户同意放宽最终地图完整度：当所有剩余有效边界相连未知区域，每个四邻域连通分量≤0.10 m²、总面积≤0.30 m²且不延伸出搜索范围或地图边缘时，允许从ACTIVE/RETRYING/BLOCKED进入原有10秒/0.1%地图稳定检查。探索器暂停导航但继续每秒重新提取边界，协调器复核当前全部边界面积；若重新超限则恢复探索。底盘在等待时HOLD；传感器/TF有效、取消、1200秒超时、地图逐格保存核对、AMCL和A/B/home规划条件保留。原始BLOCKED和失败数继续记录，并单独记录residuals面积、容许原因及exploration_completion，不把未知格改为空闲。

修改前两端相关源码备份至各自 `ROS/backups/before_residual_tolerance_20261022/`。本地一次汇总检查通过，新增小区域、重复边界去重、单块超限、合计超限、触及地图边缘、缺失证据及稳定/取消/观测失效断言；冻结内容直接比较通过。日志 `ROS/logs/mode2_debug_20261022/residual_source_check.log`。源码同步后直接内容比较一致，虚拟机 `catkin_make -j2 -l2` 退出码0，日志 `residual_build.log`。

`residual_mapping` 从初始场景运行后，剩余未知区实测0.07125 m²，符合面积限制，但原第一版入口只检查BLOCKED，导航仍在ACTIVE/恢复中，未收尾。该轮已中断并保留日志。随后将面积完成条件用于ACTIVE/RETRYING/BLOCKED，暂停目标后仍持续检查新鲜边界，不等待导航重试耗尽。

`residual_early_mapping` 从初始场景运行：仿真345.320秒开始暂停，355.444秒实测剩余未知区0.08625 m²；358.050秒保存地图、358.912秒进入定位、360.212秒观察到AMCL发布map→odom、361.798秒进入搬运。A台导航和PID交接通过，459.298秒在SELECT_BOX失败：未在A台超时内检测到红盒。地图为 `~/.ros/final_project/maps/20261022_235707_435337/map.yaml`，run.json明确记录small_residuals_accepted与最终failed；没有抓取、放置或返航。该轮已经停止，失败相机帧及日志保留在 `ROS/logs/mode2_debug_20261022/residual_early_mapping/`。

## 2026-10-23：YOLO文本提示修复

原A台相机帧中，旧 `box / cube` 提示的最高置信度约0.062，低于0.25；整帧及10个裁剪输入均未检出目标。用户授权调整提示并重新导出同一预训练权重，不训练、不换权重。两端原ONNX、检测与导出脚本备份至 `ROS/backups/before_yolo_prompts_20261023/`。

仅将第二类提示改为 `cube shaped object`，保留 `box`、640输入、两类别输出布局及0.25阈值。同一失败帧在PyTorch、导出的ONNX及虚拟机ONNX Runtime上均检出四色盒；虚拟机结果为红0.329236、蓝0.338337、黄0.309463、绿0.403840，掩码内HSV颜色均正确。模型和两脚本同步后直接内容比较一致。一次针对性检查覆盖修改脚本UTF-8/LF与Python语法、ONNX输出维度、四颜色结果、置信度及非空实例掩码；未修改共享机械臂、导航参数或场景。证据为 `ROS/logs/mode2_debug_20261022/prompt_onnx_check.json`、`prompt_onnx_check.jpg` 和虚拟机 `prompt_onnx_vm_check.json`。该检查只证明失败帧识别修复，不能替代完整流程成功。

新模型的首轮完整尝试 `prompt_mapping_20261023` 在仿真504.356秒已观测到EXHAUSTED、边界0、失败0，但随后EXHAUSTED/PAUSED反复切换，未能进入保存。根因是协调器对普通EXHAUSTED也设置exploration_hold，探索器下一次发布PAUSED；零边界没有残区证据，因此稳定窗口被重置。保留该轮日志并停止后，将内部暂停仅用于残区接受，普通EXHAUSTED继续由底盘HOLD等待稳定；不改完成门槛。两端原协调器备份至 `ROS/backups/before_exhausted_hold_fix_20261023/`。修改文件编码、语法及虚拟机实际Python 2编译检查通过；新的从头尝试为 `prompt_handoff_mapping_20261023`。

`prompt_handoff_mapping_20261023` 从初始场景运行，探索期间一次无进展失败经原重试恢复，旧失败随后解除。仿真288.658秒接受残区，合计0.125 m²、最大单块0.065 m²；303.484秒保存并核对地图，304.500秒切换定位，305.950秒观察到AMCL的map→odom，307.468秒进入搬运。现场确认GMapping已退出，搬运恢复原矩形footprint、局部栅格0.05 m、膨胀0.35 m、min_vel_x=-0.06和原恢复参数。最终401.668秒在NAV_PICK失败：DWA执行恢复后仍无有效控制，PID通道检查持续受阻。没有进入SELECT_BOX，没有抓取、放置或返航；本轮仿真已停止。

地图和run.json为 `~/.ros/final_project/maps/20261023_003537_963096/`。失败后只读检查表明，原A台导航接近点y=0.43距全局/局部障碍栅格中心约0.308/0.291 m，部分底盘转向角发生栅格碰撞；候选y=0.30的距离约0.438/0.417 m，make_plan返回非空路径。候选点仅做只读规划，尚未改动主线配置或实际行驶验证；已按用户要求就调整模式2的A台接近点请求确认。完整取送仍未通过。日志、地图、现场快照与接近点检查保留在 `ROS/logs/mode2_debug_20261022/prompt_handoff_mapping_20261023/`。

用户随后允许仅后移A台接近点，并要求不增加复杂苛刻的指标。两端原automatic.yaml备份至 `ROS/backups/before_a_approach_20261023/`，仅将A台导航y从0.43改为0.30；B台及其余配置保持原值。一遍必要检查通过UTF-8/LF、YAML解析及直接内容比较，确认只有该数值改变；同步后两端直接内容一致。新的从头运行目录为 `a_approach_mapping_20261023`，不新增完成或抓放门槛。

该轮在仿真385.994秒保存地图、387.634秒切换定位、388.974秒观察到AMCL的map→odom、390.504秒进入搬运。A台导航通过，433.604秒在线YOLO选中红盒 `delivery_box_red`（置信度0.526465、标记9）；458.800秒在DOCK_PICK_PID失败，原因是红盒标记位姿不可用。地图为 `~/.ros/final_project/maps/20261023_005047_157694/map.yaml`。现场5秒11条标记消息均只有8/10/11，没有9；不是全部标记或TF失效。该轮已保存相机、检测、地图、任务结果及日志并停止，没有抓取、放置或返航。

独立读取同一失败图像的ALVAR诊断复现：原 `max_new_marker_error=0.08` 检出10/8/11，0.12检出9；将图像放大两倍而保持0.08也检出9。红盒纹理内容与旋转方向核对正确，未翻转纹理。诊断工具首次因缺少ROS初始化未完成，补齐初始化后在独立临时主节点运行，退出后已停止该主节点；诊断不向底盘发布控制。证据为 `a_approach_mapping_20261023/marker_probe.json`、`native_alvar_probe.txt` 及调试目录的 `marker_reflection.json`。

用户随后确认仅模式2将 `max_new_marker_error` 调至0.12。两端原launch备份至 `ROS/backups/before_mode2_marker_error_20261023/`，主线只改这一模式条件参数；模式1/3保持0.08，YOLO门槛、标记尺寸、跟踪误差、视觉PID和抓放条件不变。一次必要检查通过UTF-8/LF、XML及单项直接内容比较，虚拟机实际展开三种模式确认值分别为0.08/0.12/0.08，见 `ROS/logs/mode2_debug_20261022/marker_config_check.txt`。本次仅改launch，无需重新编译节点。从头验证的新目录为 `marker_mapping_20261023`，最终结果待本轮运行记录。

`marker_mapping_20261023` 在仿真702.334秒因BLOCKED结束，剩1个未解决失败；主要边界相连未知区实测0.1725 m²，大于当时单块0.10 m²上限。该轮未保存最终地图、未交接AMCL、未到A台，因此不能据此认定ALVAR在线修复已通过。失败run.json为 `~/.ros/final_project/maps/20261023_011824_938334/run.json`；地图快照、残区测量及日志保存在同名调试目录，仿真已停止。

用户确认只将单块未知残区上限从0.10改为0.20 m²，合计0.30 m²、10秒稳定窗口、避障、定位及抓放条件保持不变。两端原配置备份至 `ROS/backups/before_residual20_20261023/`。一次必要检查通过UTF-8/LF、YAML和单项直接内容比较，源码同步后直接内容一致。本次不改探索算法或新增指标，新一轮从头运行目录为 `residual20_mapping_20261023`，最终运行结果待记录。

## 2026-10-23：A台抓取通过、B台接近点修复与现场持箱续跑

`residual20_mapping_20261023` 运行期间虚拟机重启、ROS进程消失，日志末尾存在未完整写入内容；该轮记为中断，原样保留，不认定算法失败或成功。重新确认配置与图形环境后，启动 `residual20_restarted_20261023`。

本轮从初始场景自动执行，仿真498.180秒观测到剩余未知区合计约0.22625 m²；500.970秒进入保存、502.364秒进入定位、503.814秒观察到AMCL发布map→odom、505.384秒进入搬运。512.196秒进入A台导航，558.120秒YOLO选中红盒 `delivery_box_red`（置信度0.421961，标记9），568.414秒进入抓取，583.678秒退离，586.480秒进入B台导航。667.378秒在NAV_PLACE因Navigation PID timeout失败；协调器最终消息为空，但搬运记录与节点日志保留了明确原因。红盒当时保持夹持，没有放置或返航。最终地图及原失败run.json为 `~/.ros/final_project/maps/20261023_201837_519696/`。

B台只读规划检查：原y=0.43点距全局/局部障碍格中心约0.309655/0.289723 m，候选y=0.30约0.438984/0.416667 m；候选点得到13个路径姿态。用户允许仅将B台导航接近点后移至y=0.30，并随后明确选择保留当前仿真、只重启搬运节点、从持箱状态继续。两端修改前配置、搬运脚本和项目说明保存在 `ROS/backups/before_b_approach_resume_20261023/`，课程README另备份于本地同目录的course_README.md。

本次主线只修改automatic.yaml中的B台接近点，以及automatic_delivery.py的显式持箱续跑接入。原A→B启动路径仍包含全部阶段；source=robot分支核对本轮地图、失败或取消记录、原选盒身份、实际附着及MoveIt的payload，再通过原模型专用抓持服务的幂等请求确认没有夹持其他模型。它跳过A台导航、选盒和抓取，从收臂、静止定位更新开始执行原B台导航、对位、放置、退离与返航。不改变原导航、抓放、PID、运动学、8 mm / 0.025 rad条件、模式1/3、模型、传感器或场景。原协调器已结束时，续跑取消通过搬运action接口处理，原mode2/cancel不接管该独立续跑。

一次必要检查通过UTF-8/LF、YAML、Python语法、B台单项差异、已有算法方法直接内容比较及正常/续跑执行顺序断言；虚拟机实际Python 2编译通过，同步文件直接内容一致。本次只改Python与配置，无新增action或C++变更，未重复catkin构建。部署脚本最初因ROS主节点API返回类型使用错误而在停止旧节点前退出，修正后仅替换mission_server，未重启Gazebo/AMCL或移动模型。

续跑目录为 `ROS/logs/mode2_debug_20261022/resume_b_20261023/`。仿真1003.996秒准备、1006.590秒B台导航、1017.824秒放置对位、1026.978秒PLACE、1040.550秒退离、1043.374秒返航。1111.042秒客户端记录action状态3、success=true，消息为“Resumed held payload, placed at B and returned home”，墙钟耗时164.670秒。B台对位距离误差0.007704 m、角误差0.010230 rad，满足原门槛；放稳检查平面误差0.023996 m，满足原放置结果检查。最终快照显示红盒约位于(1.210918, 0.921360, 0.205000)，attached=false，控制模式HOLD；地图返航误差约0.044317 m/0.038828 rad，里程计线速度约1.9e-7 m/s、角速度约-7.0e-7 rad/s，处于停车状态。

首次结果采集因新诊断节点尚未收到仿真时钟而提前触发TF等待超时，补齐首个有效时钟等待后已保存快照；这不涉及重跑搬运动作。成功客户端退出时的无条件取消回调产生一次actionlib告警，结果及停车状态仍为成功；后续客户端示例已仅取消活动目标。原失败run.json、原missions记录均未覆盖，续跑以source=robot和resume_from=NAV_PLACE另行追加。日志、反馈、结果、图片和最终快照保存在上述两端目录；当前Gazebo/RViz保持运行供观察。此结果证明本轮建图至抓取及现场续跑至放置返航分别通过，未验证修复后从启动开始的一次连续全流程。

## 2026-10-23：模式2启动前自动清理旧ROS后台

用户报告原命令无法正常启动。现场旧roslaunch PID 3442及Gazebo仍在运行；20:56和20:58两次新启动日志均记录Gazebo退出255，spawn_model报“SpawnModel: Failure - entity already exists”。旧后台未结束，新节点还触发同名节点替换。已保存原日志至虚拟机 `ROS/logs/mode2_startup_20261023/before_ros_logs/`，结束旧实例。随后用户明确要求停止验证，并要求今后使用原模式2命令时自动清理旧ROS后台；本轮测试启动已发送退出请求，不继续运行验证。

按该要求新增scripts/cleanup_ros.py，在模式2机器人描述生成命令中显式调用，先于新主节点和仿真节点启动。按当前用户的进程表识别旧ROS/Gazebo/RViz、ROS包Python节点、devel节点及其子进程；保留本次清理进程及完整祖先进程链，避免杀掉新roslaunch或当前终端。依次SIGINT等待15秒、SIGTERM等待5秒、SIGKILL等待2秒；失败时中止新启动。只读roslaunch查询跳过清理。模式1/3入口、生成模型几何、传感器、SLAM及搬运算法未作改动。源码和已保存地图不删除，普通模式2重启会结束旧仿真现场，显式持箱续跑仍使用原action而不重新执行simulation.launch。

修改前文件分别备份至两端 `ROS/backups/before_mode2_autocleanup_20261023/`。已同步新增脚本、launch接入、模型生成脚本、CMake安装项和说明。本次遵从用户“不用验证”的要求，未对自动清理改动执行构建、静态测试或仿真验证；此前手动清理后的启动记录不能作为此自动清理实现的实测证据。
