# 全自动建图与自主取送、键盘／语音机械臂操作

环境：Ubuntu 18.04、ROS1 Melodic、Python 2.7、Gazebo 9。TurtleBot3 Waffle 搭载四自由度 OpenMANIPULATOR；自研代码与复用来源见 [THIRD_PARTY.md](THIRD_PARTY.md)，部署结果见 [VALIDATION.md](VALIDATION.md)。

## 1. 启动

工作区为 `/home/jiayuexie/ROS`，项目是其中的 `src/final_project` 包，共用外层 `build/devel`。

启动参数 `mode:=1` 为键盘操作，`mode:=2` 为全自动建图后自主取送，`mode:=3` 为模式1的语音版本（见第8节），默认值仍为 `2`。模式只在启动时选择；模式2启动自动清理旧ROS后台，模式1/3切换前仍先关闭旧仿真。

### 模式2：全自动建图与自主取送

模式2在启动新节点前自动清理当前用户的旧ROS进程，包括roslaunch/roscore、Gazebo、RViz、旧导航与控制节点及其子进程，也包含该用户其他旧ROS任务。先SIGINT等待15秒，再SIGTERM等待5秒，最后SIGKILL；仍未退出则中止新启动。保留当前新roslaunch及其终端，不删除日志、源码或已保存地图。`roslaunch --nodes/--dump-params` 等只读查询跳过清理；直接调用模型生成脚本而不加清理选项也保持原行为。模式1/3启动入口不启用该清理。

```bash
cd /home/jiayuexie/ROS
source /home/jiayuexie/ROS/devel/setup.bash
roslaunch final_project simulation.launch mode:=2
```

在虚拟机图形桌面终端执行时，默认同时打开 `YOLO - Color Box Detection` 独立窗口，持续显示检查相机的 YOLO 检测框、类别、HSV颜色、置信度和推理耗时，目标颜色标注 `TARGET`；默认 `target_color:=red`。图像话题为 `/final_project/vision/image_annotated`。窗口可以调整大小，无检测时显示 `0 detections`，加载模型或相机未就绪时显示等待提示。帧率取决于 CPU 推理速度。

该窗口只作观察，任务仍仅在 A 台原 `SELECT_BOX` 阶段运行自身识别并决定抓取；窗口不发控制命令，不改原任务日志或抓放条件。A 台选色期间两个推理进程并存，CPU负载增加。`yolo_view:=false` 关闭窗口及额外推理；模式1、3不启动它。无图形桌面时同时使用 `gui:=false rviz:=false yolo_view:=false`。各轮实际验证范围见 VALIDATION.md 最新记录。

YOLOE-26n-seg使用原预训练权重，固定文本提示为 `box / cube shaped object`，导出为两类别ONNX；不训练模型，检测置信度仍为0.25。对检测实例掩码内的饱和像素进行HSV判色，再按所选颜色绑定盒子模型和ALVAR标记。模式2的ALVAR新标记最大误差为0.12，模式1/3仍为0.08；标记尺寸、跟踪参数及最终抓放条件保持原值。2026-10-23实际已完成建图至A台抓红盒；B台原接近点失败后，在同一仿真中持箱续跑，完成放置、返航与停车。原失败及续跑成功分别保留，不标为修复后从启动开始一次连续成功，详见VALIDATION.md。需要重新导出时，在独立导出环境运行 `python src/final_project/vision/export_model.py <已有yoloe-26n-seg.pt绝对路径>`，同目录须预先缓存 `mobileclip2_b.ts`；运行仿真不调用导出脚本或下载模型。

一次启动自动执行：**收臂 → 自主探索与GMapping建图 → 保存最终地图 → 切换AMCL → A台导航、视觉PID对位与抓取 → 退离 → B台放置 → 退离 → 返回待命点 → 停车**。无需人工选择探索方向、点击目标、输入搬运任务或确认开始，不循环执行第二次任务。只关闭RViz可用：

```bash
roslaunch final_project simulation.launch mode:=2 rviz:=false
```

每个新终端先加载同一个工作区，初始盒子位于A台，只启动一套仿真。保留Mesa软件渲染与原清晰档传感器。2026-10-22修复代码已同步虚拟机并实际构建，完整流程的实际结果以 VALIDATION.md 最新记录为准；历史点击导航或旧搬运记录不能替代。

模式2从空白地图开始，复用explore_lite边界搜索，按1 Hz自动选择已知空闲区中的观测目标，经move_base规划与激光避障前往。8×6 m边界仅限制搜索范围，不读取World障碍布局或预制占据图作为控制输入。有效边界须同时满足累计栅格长度和点间最大几何跨度均至少0.30 m，避免小型未知斑块的弯曲边缘被反复选中；旧失败不再对应有效边界时解除。目标安全半径0.35 m，路径不得穿越未知区。

完成条件为有效可达边界耗尽且无未解决失败，或者仅剩满足面积限制的小型未知区域；两条路径都要求观测有效，且连续10秒仿真时间内固定区域的地图分类变化不超过0.1%。用户确认的面积限制为：与剩余有效边界相连的未知区域按四邻域连通搜索，每块≤0.20 m²、合计≤0.30 m²，且不得延伸出搜索边界或地图边缘。所有剩余有效边界对应区域都通过该检查时，立即HOLD并暂停探索目标；探索器继续发布新鲜地图边界，稳定窗口内持续复核面积，新边界超限则恢复探索。该判据也适用于ACTIVE/RETRYING，避免小残区仍在导航恢复时无法收尾；原始BLOCKED/失败数及面积、容许原因记入run.json，不改写未知格。较大区域受阻、缺失边界证据和ERROR仍失败。单目标60秒仿真时间无进展时清理代价地图并重试一次。总探索上限1200秒仿真时间，超时为失败停车。

完成探索后停车，冻结地图为快照，使用map_saver保存并逐格核对PGM与YAML。运行目录为 `~/.ros/final_project/maps/<运行时间>/`，包含map.pgm、map.yaml、map_saver.log和run.json。随后停止探索、GMapping和建图move_base，加载刚保存的地图、启动AMCL并初始化停止时位姿。定位和任务接近点路径检查通过后，才启动Delivery action并自动提交box1的A→B任务。

搬运节点在初始化动作客户端前等待首个有效仿真时钟，避免晚启动节点以零时刻计算超时、收到当前时钟后立即误报导航不可用。该等待仅增加模式2启动同步，不改变导航、对位或抓放算法。

模式2的A/B台导航接近点分别为场景坐标 `(-1.2, 0.30, 1.5708)`、`(1.2, 0.30, 1.5708)`，先保持转向空间，再沿用原PID接近；A台仍先运行YOLO选盒及视觉对位。两点由启动时坐标关系转换至本轮地图，不改变桌台、盒子、最终对位和抓放条件。

现有A/B点和home仍为场景坐标，通过启动时场景→SLAM地图关系转换后用于导航和台面规划。Gazebo状态只用于原任务前提、抓取条件和结果验收，不输入导航、视觉PID或定位。RViz保留四项传感器及点云并增加探索边界；模式2不接入原点击目标，也移除其人工目标/初始位姿工具。点云仍为实时RGB-D观测，不是三维融合SLAM。

观察与取消：

```bash
rostopic echo /final_project/status
rostopic echo /final_project/exploration/status
rosservice call /final_project/mode2/cancel '{}'
```

取消或失败停车，已抓物体保持附着，不自动松爪或恢复。取消整个流程使用上述服务，不单独取消move_base来模拟流程完成。成功后保留最终地图显示；重新执行完整任务须关闭整套仿真再启动。模式1/3继续使用原固定地图及AMCL。

模式2支持显式持箱续跑：同一仿真、地图、目标颜色和原YOLO所选模型的记录须匹配，实际物理夹持和MoveIt的 `payload` 附着物仍存在，最近搬运记录须在 `NAV_PLACE/DOCK_PLACE_PID/PLACE` 失败或取消。向原 `/final_project/delivery` action发送 `DeliveryGoal('box1', 'robot', 'B')`，会校验实际夹持模型，再执行收臂、静止定位更新、B台导航、原对位放置、退离及返航；不再去A台或运行选盒、抓取。若更新了脚本，只需先重启搬运节点，不重启Gazebo、AMCL或地图；不要重新执行simulation.launch，模式2普通启动会清理旧现场。原失败run.json保留，续跑另记source=robot与resume_from=NAV_PLACE；这不是从启动开始的一次完整成功。原协调器已结束时，取消续跑使用搬运action的cancel接口。

### 模式1：键盘执行同一搬运任务

第一终端启动模式1：

```bash
source /home/jiayuexie/ROS/devel/setup.bash
roslaunch final_project simulation.launch mode:=1
```

等待 `READY MODE1`，在虚拟机的第二个交互终端运行，并保持键盘焦点在该终端：

```bash
source /home/jiayuexie/ROS/devel/setup.bash
rosrun final_project keyboard_control.py
```

| 按键 | 操作 |
| --- | --- |
| `R` | 收臂到原载物姿态、静止更新定位，成功后允许驾驶；取消或臂动作失败后用此键恢复 |
| `W/S` | 前进 / 后退，需持续按键或键盘自动重复 |
| `A/D` | 原地左转 / 右转 |
| `T` | 切换细调速度；普通为前进 0.1 m/s、后退受原限幅为 0.08 m/s、转向 0.3 rad/s，细调为 0.02 m/s、0.08 rad/s |
| `1/2` | 选择操作台 A / B；这是地点选择，不是运行中切换模式 |
| `P` | 对准后，执行原有完整抓取：开爪、接近、闭爪、条件约束、抬升、收臂 |
| `O` | 对准后，执行原有完整放置：下降、释放、开爪、收臂、放稳检查 |
| 空格 | 立即请求停车，取消当前键盘臂动作；已夹持物品保留约束 |
| `F` | 完成取送并手动返航后，检查待命点位置和航向，记录本次任务完成 |
| `Q` / `Ctrl+C` | 停车、取消当前键盘臂动作并退出键盘节点 |

流程为 `R → 手动驾驶至 A 台前 → 1、T、细调、P → 手动退离 → 手动驾驶至 B 台前 → 2、细调、O → 手动退离并返航 → F`。三种模式共用8×6 m迷宫，中央纵墙将A/B分隔，换侧需从墙南端或北端绕行，不能穿墙直达。待命点为地图坐标 `(0, -1.2)`、航向 `1.5708 rad`；返航完成要求沿用原流程的 `0.09 m / 0.1 rad` 校正后验收条件。

终端实时显示所选台、速度档、夹持状态、目标距离误差 `dx`、方位误差 `a` 和距待命点的距离 / 航向误差。`V` 为新鲜物体视觉标记，`TF` 为所选台的已知目标；无新鲜标记时明确显示提示。按 `P/O` 前，应将 `|dx|` 调至小于 `0.008 m`、`|a|` 小于 `0.025 rad`。抓放键会先停车、静止更新定位，再重新校验；不满足条件会拒绝并提示继续手动调整，不会自动驾驶或 PID 对位。

三种模式复用同一盒子、A/B台、机械臂规划、IK/FK、条件约束抓取和放稳验收。模式1的行驶、对位、退离和返航均由驾驶员负责；机械臂动作期间拒绝键盘行驶，只有确认收臂后恢复驾驶。键盘输入停止约0.25秒后发布零速度，速度仲裁另设0.3秒手动指令超时，原激光过期和前后近距离停止检查仍有效。

模式1完成后按F、模式3说“小车完成”，仍写入原 `missions.jsonl` 并记录实际模式。只启动一个键盘或语音控制节点。

## 2. 备份与复用边界

本地完整备份：`F:/University/Course/dasanshang/ROS_class/ROS/backups/before_mode2_slam_20261019/final_project_with_autonomous_delivery.zip`。

虚拟机完整备份：`/home/jiayuexie/ROS/backups/before_mode2_slam_20261019/final_project_with_autonomous_delivery.tar.gz`，保留Linux执行权限。两份归档均含改造前完整包与历史日志，位于src之外，不参与catkin构建。

本轮改造前完整版本另存于 `ROS/backups/before_mode2_auto_mapping_20261020/`，包含完整包与课程README，保留上述旧备份。

本轮仅从旧搬运备份恢复Delivery.action、导航/PID交接、视觉对位和退离；模式2专用派生类继承当前完整抓放并顺序编排，不恢复SMACH、task.py、command_router.py、send_text.py及文本/语音确认路由。模式1、3共用抓放、IK/FK、8 mm / 0.025 rad校验及静止AMCL修正保留。历史旧搬运命令仍不可执行。

恢复前先停止仿真，将当前 `src/final_project` 移到src之外，再把备份解压到src并重新构建。不要在src中保留第二个同名包；包外的私有讯飞凭证和Python语音环境不随备份分发。

## 3. 运行接口

| 接口 | 类型与用途 |
| --- | --- |
| `/map` | 模式2先实时SLAM，后为本轮保存地图；模式1/3为原固定地图 |
| `/move_base_simple/goal` | 模式2不接入；自动目标通过move_base action提交 |
| `/move_base/status`、`/move_base/result` | 原生导航状态和结果，不可达目标会报告失败 |
| `/move_base/cancel` | `actionlib_msgs/GoalID`，取消导航 |
| `/final_project/navigation_cmd_vel` | move_base速度，经原仲裁到达 `/om_with_tb3/cmd_vel` |
| `/final_project/manual` | 模式1/3的 `ManualAction`，`prepare/pick/place/finish`，地点A/B |
| `/final_project/manual_cmd_vel` | 模式1键盘或模式3语音的手动速度 |
| `/final_project/status` | 流程阶段、失败原因、最终结果或手动动作反馈 |
| `/final_project/control_mode` | `HOLD/NAV/DOCK/MANUAL`；模式2允许NAV/DOCK，模式1/3仅允许MANUAL |
| `/final_project/set_grasp`、`/final_project/attached` | 原条件约束抓取服务和状态 |
| `/ar_pose_marker` | 原 `ar_track_alvar_msgs/AlvarMarkers` 视觉观测 |

`/final_project/delivery`（DeliveryAction）仅在模式2最终地图定位就绪后提供，由协调节点自动调用。`/final_project/mode2/cancel`为Trigger取消服务；`/final_project/exploration/status`为JSON内容的String，包含state、frontiers、未解决failures数量和frontier_points地图坐标。旧command/dialogue及SMACH接口不恢复，模式3voice_command保持原样。

## 4. SLAM、定位与避障

模式2子进程启动/停止均在主线程执行。探索时仅GMapping发布map→odom；停止后才启动map_server、AMCL及搬运move_base。保存时的SLAM位姿初始化AMCL，等待新鲜位姿、TF及八次静止更新，协方差对角项须不超过0.04，交接修正不超过0.5 m / 0.5 rad；失败保持HOLD。模式1/3保持原AMCL。TF链和Gazebo上游world模拟里程计保留，不宣称实体精度。

GMapping仍采用3.5 m激光、0.025 m栅格及30粒子。模式2协调器将原 `/scan` 中仅正无穷的无回波读数转换为 `range_max`，通过 `/final_project/mode2/slam_scan` 供GMapping清除至3.45 m的已知空闲空间；不在最大距离生成假障碍，原扫描、导航激光及传感器配置不变。扫描对应TF最多等待0.1秒，原新鲜度门槛不变。实时点云保持640×480、10 Hz配置目标，实际速率受虚拟机性能限制。

仅建图阶段禁止DWA倒车（`min_vel_x=0`），保留原后向0.29 m激光保护；启用15秒/0.10 m振荡检测与原生带碰撞检查的转向恢复。全局和局部代价地图使用外切于0.35 m圆的16边形安全包络，覆盖原底盘并为转向保留空间。切换AMCL时恢复搬运阶段原矩形包络、倒车、振荡及恢复默认值，不影响模式1/3。BLOCKED仅允许上述有实际地图面积证据的小残区例外；该面积规则也可提前结束ACTIVE探索，避障、AMCL定位和A/B/home路径检查仍保留。

建图全局/局部膨胀半径为0.50 m，保留8.0衰减系数；局部代价地图采用与SLAM一致的0.025 m栅格，为扩大后的包络提供贴墙前的代价梯度并减少转向离散误差。搬运阶段恢复原0.35 m膨胀半径及0.05 m局部栅格。

探索沿用通用0.10 m / 0.15 rad导航终点容差；地图切换后恢复原导航0.045 m / 0.05 rad及受观测、通道检查约束的近距离PID交接。抓放仍要求8 mm / 0.025 rad。Navfn/DWA使用当前地图、激光障碍层和膨胀层绕障，没有通路或对位超时则报告失败。

velocity_gate仍为唯一底盘速度发布者，保留限幅、超时和激光过期/近障停车。模式2分阶段允许NAV/DOCK，抓放期间HOLD，不接受键盘/语音驾驶；模式1/3仍由原手动动作管理MANUAL。

## 5. 机械臂运动学与抓取

四自由度机械臂采用适合本场景的水平夹持约束：末端俯仰为零，`q4=-(q2+q3)`。`horizontal_ik` 从实际展开的 URDF 读取连杆偏移和关节限位，求解两个肘部分支并筛选可行解；每次调用 MoveIt FK 回代，位置残差超过 2 mm 时拒绝执行。

MoveIt/OMPL 对关节目标做碰撞规划，真实轨迹通过 `FollowJointTrajectory` 控制器执行并检查结果。桌面在底盘停止后转换到本体规划坐标系更新。上游 KDL 配置也保留为标准运动学接口；实际搬运使用满足水平姿态约束的解析解，避免四自由度位置 IK 忽略夹爪方向。

抓取流程为张爪、预抓取、接近、闭爪、条件约束、抬升、收臂。夹指碰撞几何向工具下方延伸约 31 mm，因此工具目标比盒子中心高 12 mm，避免水平夹指穿入台面；附着物体与放置目标使用同一偏移。初始稳定视觉结果在底盘 `HOLD` 时冻结，允许手臂在最后接近阶段遮挡顶部标记。

采用用户确认的**条件约束辅助抓取**：插件在 Gazebo 物理更新线程检查物体存在、两侧夹指的实际滑动关节位置均不超过 8 mm（张开值为 19 mm）及工具到物体距离不超过 35 mm，再建立真实 fixed joint。闭合指令为 3 mm，校验容差允许盒子接触引起的夹指偏差。物体不是通过持续写入 Gazebo 位姿跟随机器人。松开时解除约束，物体按重力和接触求解落到台面；取起与放稳均检查仿真模型状态。

该方式验证移动操作流程，不宣称已验证纯摩擦夹持。Gazebo 模型状态仅用于抓取条件与结果验收，不替代视觉定位和 AMCL。

## 6. 环境与复现

已存在 Melodic 时，可运行项目配置脚本安装仿真依赖并统一构建：

```bash
bash /home/jiayuexie/ROS/src/final_project/tools/configure_vm.sh
source /home/jiayuexie/ROS/devel/setup.bash
```

脚本不升级操作系统、不配置语音。该虚拟机约 4 GB 内存，构建使用 `-j2 -l2`。资源再生工具 `tools/prepare_assets.py` 在 Windows 使用用户的本地参考目录执行，产物已随源码提供，虚拟机不需要访问 Windows 路径。

实际结果见 [VALIDATION.md](VALIDATION.md) 最新章节。模式2待验收自动探索、地图交接与完整取送返航；任务记录仍为 `~/.ros/final_project/missions.jsonl`。旧自主搬运和点击导航记录仅为历史。

本机离线检查入口（Python 3；不导入ROS、不启动仿真）：

```powershell
& 'E:\Programming\mniconda3\python.exe' -X utf8 'F:\University\Course\dasanshang\ROS_class\ROS\src\final_project\tools\check_project.py' --static-only
```

检查编码、语法与Python 2兼容写法、配置和引用，执行少量完成/受阻、稳定窗口、坐标、保存像素和取消保护断言，并与改造前备份直接比较冻结内容。明确跳过Python 2实际执行、ROS导入、URDF展开、插件加载、C++/catkin构建及仿真；后续虚拟机验收步骤见VALIDATION第12节。

## 7. 全模式传感器与迷宫

依据课程“URDF、Gazebo与Rviz综合应用”的四项要求及 [Autolabor 6.7](https://www.autolabor.com.cn/book/ROSTutorials/di-6-zhang-ji-qi-ren-xi-tong-fang-zhen/67.html) 实现。模式1/2/3共用机器人、World与传感器显示；模式2先使用实时SLAM、后使用本轮保存地图，模式1/3使用原固定地图，`rviz:=false`只关闭显示，不关闭传感器。

| 模块 | 话题 | 坐标系 / RViz |
| --- | --- | --- |
| 运动 / 里程计 | `/om_with_tb3/cmd_vel`、`/odom` | `odom → base_footprint`；Robot、Odometry 箭头 |
| 激光 | `/scan` | `base_scan`；Laser |
| 普通相机 | `/inspection/image_raw`、`/inspection/camera_info` | `inspection_camera_optical_frame`；Inspection camera |
| Kinect RGB | `/kinect/rgb/image_raw`、`/kinect/rgb/camera_info` | `camera_rgb_optical_frame`；Kinect RGB |
| Kinect深度 | `/kinect/depth/image_raw`、`/kinect/depth/camera_info` | 同上；Kinect Depth（自动归一化灰度） |
| Kinect点云 | `/kinect/depth/points` | 同上；Kinect PointCloud（RGB着色） |

这里是 **R200 外形上的 OpenNI Kinect 兼容仿真**，复用已有光学 TF（Z向前、X向右、Y向下），不代表真实 Kinect 硬件。`libgazebo_ros_openni_kinect.so` 由 `gazebo_plugins` 提供。RGB-D 为 640×480、10 Hz、水平视场约60°、裁剪0.05–8 m，点云有效范围0.4–8 m，RGB/深度共用光学系且标定投影为零双目偏移；普通俯视相机保持1280×960、10 Hz供原抓放使用。以上为配置值，实际帧率取决于虚拟机性能。RViz目标15 FPS，不据配置宣称实测达到15 FPS。

Odometry仅绘制位姿与方向箭头。差速插件对未观测的Z/横滚/俯仰使用极大协方差，因此关闭其协方差图形，避免巨大的椭球遮挡整个视图；原里程计消息与数值保持不变。

世界外墙位于 x=±4、y=±3，高1 m；左右横墙中心为(±2.65,-0.4)，长2.6 m；中央墙中心为(0,0.7)，沿Y长2.2 m；圆柱位于(±2.7,-1.7)、(±2.7,1.5)，半径0.18 m、高0.8 m。保留A/B台和初始盒子，移除旧中央方障碍。World中的视觉与碰撞形状一致；静态占据地图由同一矩形/圆形列表生成，0.025 m分辨率、原点(-4.5,-3.5)、360×280栅格。仅重建资源时在Windows运行：

```powershell
& 'E:\Programming\mniconda3\python.exe' -X utf8 'F:\University\Course\dasanshang\ROS_class\ROS\src\final_project\tools\prepare_assets.py' --reference 'F:\University\Course\dasanshang\ROS_class\参考'
```

验收时检查里程计变化、激光贴合墙柱、图像更新、米制深度与点云方向。模式2还须检查无人工目标的探索、最终地图保存、互斥定位交接、绕障及同次取送返航；无通路时停止并报告。模式1/3抓放及返航条件保留。

## 8. 模式3：语音驾驶与完整抓放

模式3是模式1的语音版本，不会自主规划行驶、对位或返航。模式1使用键盘，模式2自动建图并自主取送；默认启动模式仍为 `mode:=2`。三种模式共用场景与传感器；讯飞的当前配置与既有验收结果见本节及 `VALIDATION.md` 第10节。

### 配置与启动

一次性配置隔离 Python 3.8 环境，不替换 ROS Python 2：

```bash
bash /home/jiayuexie/ROS/src/final_project/tools/configure_voice.sh
```

讯飞须开通“语音听写（流式版）WebAPI”，不是 Spark 对话接口。私有配置文件为 `~/.config/final_project/xfyun.env`，填写 `XF_APP_ID`、`XF_API_KEY`、`XF_API_SECRET` 三项并执行 `chmod 600 ~/.config/final_project/xfyun.env`。凭证不放在源码、ROS 参数或启动命令中；本次授权配置由部署过程完成，不在文档展示其值。

Windows 麦克风经 VMware 虚拟声卡进入 Ubuntu，默认使用 ALSA `pulse`。SSH 不转发麦克风；先在 Ubuntu 确认输入设备实际能录到说话声，不使用输出 monitor 源。

```bash
source /home/jiayuexie/ROS/devel/setup.bash
roslaunch final_project simulation.launch mode:=3
# 需要其他已确认的 ALSA 录音设备时指定 voice_device:=设备名
```

等待 `READY MODE3` 和 `VOICE_LISTENING`，说一句“小车准备”。收臂成功后再驾驶，不需要键盘控制节点。语音控制退出后仿真继续运行；如需重新启动语音控制，关闭并重新启动整套模式3仿真，不运行中切换模式。

### 指令

每句以“小车”开头，只说一条完整指令，说完留出约3秒安静时间。连续重复可能合并成一句并被拒绝。允许空格、A/B大小写和句末常用标点；不做关键词包含匹配。“小车不要前进”“小车前进然后抓取”均拒绝，不续期已有运动。

| 语音 | 行为 |
| --- | --- |
| 小车准备 | 原 `prepare`：收臂、更新定位，成功后允许驾驶 |
| 小车前进／后退／左转／右转 | 当前档位连续运动，接受新方向即替换；10秒无新有效方向即停车 |
| 小车停止／取消 | 清零速度，取消正在执行的手动动作；保留已夹物体的约束 |
| 小车普通／细调 | 停车并选择速度档；原普通0.1 m/s、0.3 rad/s，细调0.02 m/s、0.08 rad/s；后退仍受原0.08 m/s限幅 |
| 小车微调前进／后退／左转／右转 | 原细调速度点动0.25秒后停车，不改变当前档位；约5 mm或0.02 rad为指令量，不保证实际位移相等 |
| 小车选择A台／选择B台 | 停车、选择操作台，不导航 |
| 小车抓取／放置 | 停车并调用所选台的原完整 `pick/place`；不自动对位 |
| 小车完成 | 原 `finish`，在用户返航后检查，不自动返航；成功记录 `operation_mode=3` |
| 小车状态 | 显示操作台、档位、夹持、视觉/TF对位误差及任务状态 |
| 小车退出 | 停车、取消当前动作，关闭语音输入，保留仿真 |

流程：准备→驾驶至A→选择A台、微调、抓取→后退绕墙→驾驶至B→选择B台、微调、放置→后退返航→完成。抓放仍要求原8 mm / 0.025 rad对位；未满足会拒绝，不能通过语音绕过。机械臂动作期间不接受移动、不排队，完成后重新说方向；取消后说“小车准备”恢复。

`/final_project/voice_command` 为最终文本（`std_msgs/String`），`/final_project/voice_status` 为接收结果、对位提示与语音状态。控制节点20 Hz发布原手动速度；原0.3秒速度仲裁超时和激光停止条件保留。识别、录音或语音子进程异常会停车并取消当前手动动作，故障后不自动恢复旧速度。

语音实时输入为16 kHz单声道S16_LE，云端句尾检测600 ms，合并动态修正且仅执行最终文本。静音和正常会话轮换不发指令。云端存在识别延迟，“停止”不是本地即时按钮，精对位使用微调。未新增语音合成或关节逐个控制。

### 文本验收与真实语音的区别

只验证控制链时可明确关闭录音：

```bash
roslaunch final_project simulation.launch mode:=3 voice_audio:=false
# 另一终端先 source 同一工作区
rostopic pub -1 /final_project/voice_command std_msgs/String "data: '小车准备'"
rostopic echo /final_project/voice_status
```

`voice_audio:=false` 不访问讯飞，不能作为麦克风或在线识别通过的证据。模式2不再提供原自主搬运文本或语音路由。

模式3此前已通过静态检查、构建、启动、文本控制与参考PCM在线识别；这些是模式2改造前记录。后续真实麦克风验收已通过连续识别，以及“状态→准备→微调前进→停止”的ROS控制链；已修复Python 2中文日志导致语音节点退出的问题。微调经原仲裁输出0.02 m/s，约0.251秒后归零。用户已要求保存结果并停止测试，录音与测试仿真均已停止。

完整语音A→B→返航、运动中的语音停车和说话结束至执行的云端延迟尚未验收。上述短流程不替代完整搬运，详细结果见 `VALIDATION.md` 第10节。
