# ROS1 Melodic 导航、在线语音与移动机械臂开源复用调研

检索日期：2026-10-16。目标环境：Ubuntu 18.04、ROS1 Melodic、Gazebo；底盘与机械臂型号未限定，语音使用讯飞在线接口。

本报告依据候选仓库的实际 README、分支、launch、包清单和关键源码整理。已核对的能力与拟议的集成方案分别说明；未在用户虚拟机中安装、编译或试跑这些新候选项目。没有找到可据此承诺“原样下载即可完成全部需求”的一体化仓库，也无法保证课程成绩。

## 1. 选型建议

优先采用 **TurtleBot3 Waffle + OpenMANIPULATOR + MoveIt + SMACH + 讯飞在线语音适配层**，项目主题建议为“语音驱动的移动机械臂自主取送”。

理由：ROBOTIS 的 Melodic 源码已经同时提供机器人模型、Gazebo 房间、定位导航、MoveIt 配置和导航抓放状态机，能复用一套匹配的移动操作平台。需要补写的重点是语音到任务的转换、任务参数化、执行结果判断和失败恢复。

| 方案 | 已核对的 Melodic 依据与能力 | 适合用途 |
| --- | --- | --- |
| ROBOTIS TurtleBot3 + OpenMANIPULATOR | 两个仓库均有 `melodic-devel`；含 Gazebo、导航、MoveIt、SMACH 抓放状态机 | 主推荐，作为统一集成基座 |
| 六部工坊 `wpb_mani` | `main` 的 README 明确 Ubuntu 18.04/Melodic；仓库内有仿真、导航、FK/IK 和抓取示例 | 中文教学资料优先时的替代平台 |
| Fetch | `gazebo9` 的 README 明确对应 Ubuntu 18.04/Melodic；有导航和七关节机械臂抓放示例 | 对机械臂姿态控制要求更高时的替代平台 |

选择其中一个平台完成闭环。不同平台的 URDF、关节名、控制器、相机话题和抓取接口需要各自配套，不能直接交叉拼接。

## 2. 主推荐：ROBOTIS 移动机械臂

### 2.1 应复用的仓库与分支

- [turtlebot3_manipulation / melodic-devel](https://github.com/ROBOTIS-GIT/turtlebot3_manipulation/tree/melodic-devel)：模型、工具、MoveIt 和任务状态机。
- [turtlebot3_manipulation_simulations / melodic-devel](https://github.com/ROBOTIS-GIT/turtlebot3_manipulation_simulations/tree/melodic-devel)：Gazebo、房间场景和控制配置。
- [ROBOTIS 官方移动操作文档](https://docs.robotis.com/docs/systems/turtlebot3/manipulation/)：配套说明；执行 Melodic 命令时应以对应分支源码为准。

两个仓库的 Melodic 分支许可证为 Apache-2.0。仓库改过名称，**ROS 包名仍然使用 `open_manipulator_with_tb3_*`**。当前默认分支不能替代 `melodic-devel`。

### 2.2 已核对的具体入口

| 功能 | 原始文件 |
| --- | --- |
| Gazebo 房间、机器人生成、关节控制器 | [gazebo/launch/rooms.launch](https://github.com/ROBOTIS-GIT/turtlebot3_manipulation_simulations/blob/melodic-devel/open_manipulator_with_tb3_gazebo/launch/rooms.launch) |
| 导航与机械臂的组合启动 | [tools/launch/rooms.launch](https://github.com/ROBOTIS-GIT/turtlebot3_manipulation/blob/melodic-devel/open_manipulator_with_tb3_tools/launch/rooms.launch) |
| 定位导航 | [tools/launch/navigation.launch](https://github.com/ROBOTIS-GIT/turtlebot3_manipulation/blob/melodic-devel/open_manipulator_with_tb3_tools/launch/navigation.launch) |
| 抓放任务启动 | [tools/launch/task_controller.launch](https://github.com/ROBOTIS-GIT/turtlebot3_manipulation/blob/melodic-devel/open_manipulator_with_tb3_tools/launch/task_controller.launch) |
| 导航、AR 目标接近与抓放逻辑 | [tools/nodes/pick_and_place_state_machine](https://github.com/ROBOTIS-GIT/turtlebot3_manipulation/blob/melodic-devel/open_manipulator_with_tb3_tools/nodes/pick_and_place_state_machine) |
| 机械臂逆运动学配置 | [waffle_moveit/config/kinematics.yaml](https://github.com/ROBOTIS-GIT/turtlebot3_manipulation/blob/melodic-devel/open_manipulator_with_tb3_waffle_moveit/config/kinematics.yaml) |

已确认的实现关系：

- `tools/rooms.launch` 同时包含操作与导航启动文件，使用 `arm` 规划组，默认机器人命名空间为 `om_with_tb3`。
- `pick_and_place_state_machine` 使用 SMACH，导航通过 `MoveBaseAction`，操作通过机械臂服务，已有“导航 → 接近目标 → 抓放”的组织基础。
- 机器人型号参数支持 `waffle`、`waffle_pi`。主方案选 `waffle`，后续配置应始终一致。
- Gazebo `rooms.launch` 的 `paused` 默认值为 `true`，演示时需要显式解除暂停。
- `kinematics.yaml` 使用 KDL，并启用 `position_only_ik: True`。

依赖准备好以后，对应的原始启动入口如下，每组在独立终端中运行，并先加载同一工作区环境。这些命令是从源码确认的入口，本轮没有试跑：

```bash
# 终端 1：Gazebo
source /home/jiayuexie/ROS/devel/setup.bash
export TURTLEBOT3_MODEL=waffle
roslaunch open_manipulator_with_tb3_gazebo rooms.launch paused:=false
```

```bash
# 终端 2：导航与机械臂配置
source /home/jiayuexie/ROS/devel/setup.bash
export TURTLEBOT3_MODEL=waffle
roslaunch open_manipulator_with_tb3_tools rooms.launch
```

```bash
# 终端 3：上游原始任务示例；完成初始定位并检查场景后运行
source /home/jiayuexie/ROS/devel/setup.bash
roslaunch open_manipulator_with_tb3_tools task_controller.launch
```

这不是完整安装清单：机器人基础包、感知包、MoveIt、控制器等传递依赖仍需在部署阶段按 Melodic 版本解析。

### 2.3 需要修改或验证的部分

- 上游状态机含固定房间目标和抓放参数，任务不能直接由任意语音指令驱动。应把地点、物体标识和目标位姿改成配置或任务输入。
- 已阅读的接近目标逻辑存在无限循环、标记丢失时的运动逻辑以及缺少有界重试的问题。集成时需要补充超时、取消、目标有效性和失败出口。
- 机械臂为四自由度加夹爪。应设计在其可达空间内、方向受约束的桌面抓取；不能承诺任意六维末端位姿都能求解。
- MoveIt 场景中的附着对象与 Gazebo 中物体实际被夹起是两个验证项目。必须检查物体在 Gazebo 中随夹爪移动、运输后保持、松爪后落稳，才能报告物理仿真抓取成功。

## 3. 中文替代方案：wpb_mani

仓库：[6-robot/wpb_mani](https://github.com/6-robot/wpb_mani)。已核对 `main`，根许可证为 BSD-3-Clause，示例源码也保留 BSD 许可说明。

README 明确支持 Melodic/Ubuntu 18.04，平台为麦克纳姆全向底盘、四自由度机械臂，适合课程展示移动、建图、定位导航和机械臂运动学。

| 功能 | 已阅读的源码入口 |
| --- | --- |
| Gazebo + 地图 + AMCL + move_base | [simulator/launch/navigation.launch](https://github.com/6-robot/wpb_mani/blob/main/wpb_mani_simulator/launch/navigation.launch) |
| 仿真抓取场景与行为启动 | [simulator/launch/grab_box.launch](https://github.com/6-robot/wpb_mani/blob/main/wpb_mani_simulator/launch/grab_box.launch) |
| Gazebo 机械臂规划入口 | [simulator/launch/moveit.launch](https://github.com/6-robot/wpb_mani/blob/main/wpb_mani_simulator/launch/moveit.launch) |
| MoveIt 逆运动学示例 | [tutorials/src/wpb_mani_arm_inverse_kinematics.cpp](https://github.com/6-robot/wpb_mani/blob/main/wpb_mani_tutorials/src/wpb_mani_arm_inverse_kinematics.cpp) |
| 抓取行为与简化几何计算 | [behaviors/src/wpb_mani_grab_box.cpp](https://github.com/6-robot/wpb_mani/blob/main/wpb_mani_behaviors/src/wpb_mani_grab_box.cpp) |

实际检查发现：

- 导航使用 `global_planner/GlobalPlanner` 和仓库内的 `wpb_local_planner/WpbLocalPlanner`，不是默认 DWA。
- IK 示例调用 `RobotState::setFromIK`，规划组为 `wpb_arm`，末端为 `end_effector_link`；该例向 `/wpb_mani/joint_ctrl` 发送结果。
- 抓取行为通过 `/wpb_mani/grab_box` 接收位姿，通过 `/wpb_mani/grab_result` 输出完成信息；多个动作阶段使用固定计数延时，不能据此判断抓取确实成功。
- 名为 `tutorials/launch/mobile_manipulation.launch` 的文件启动串口底盘、RPLidar 和 Azure Kinect，并引用 `/home/nvidia/waypoints.xml`，是硬件流程，不能直接当成用户的仿真总入口。
- 导航和抓取示例各自启动 Gazebo，并使用不同 world。集成时应保留一个 world、一套机器人和 TF，再组合功能节点。

结论：中文资料和单项教学内容很匹配，完整语音自主取送仍需集成。其四自由度机械臂也需要遵守可达空间和抓取方向约束。

## 4. 高自由度替代方案：Fetch

仓库：[fetchrobotics/fetch_gazebo / gazebo9](https://github.com/fetchrobotics/fetch_gazebo/tree/gazebo9)。README 明确 `gazebo9` 对应 Melodic/Ubuntu 18.04，不能改用面向其他发行版的分支。

已核对：

- [fetch_gazebo_demo/launch/fetch_nav.launch](https://github.com/fetchrobotics/fetch_gazebo/blob/gazebo9/fetch_gazebo_demo/launch/fetch_nav.launch)：包含导航启动和配套地图。
- [fetch_gazebo_demo/launch/pick_place_demo.launch](https://github.com/fetchrobotics/fetch_gazebo/blob/gazebo9/fetch_gazebo_demo/launch/pick_place_demo.launch)：启动 MoveIt、`simple_grasping` 感知节点与抓放脚本。
- [fetch_gazebo_demo/scripts/pick_place_demo.py](https://github.com/fetchrobotics/fetch_gazebo/blob/gazebo9/fetch_gazebo_demo/scripts/pick_place_demo.py)：七个机械臂关节、抓取候选、规划场景和抓放调用，保留 Python 2 语法。
- [fetch_gazebo_demo/package.xml](https://github.com/fetchrobotics/fetch_gazebo/blob/gazebo9/fetch_gazebo_demo/package.xml)：声明 BSD；抓放脚本含 BSD 三条款许可文本。应按复用文件及其依赖分别保留许可证。

重要限制：当前抓放脚本中的底盘导航调用被注释，部分动作循环也没有有界失败退出。它可复用成熟的感知和操作接口，但不能被描述为已完成导航搬运的即用系统。相较四自由度方案，其感知、规划和仿真配置更多，是否适合当前虚拟机资源仍待实测。

## 5. 讯飞在线语音：优先复用官方协议与 SDK

### 5.1 原始资料

- [讯飞语音听写流式 WebAPI 官方文档](https://www.xfyun.cn/doc/asr/voicedictation/API.html)：已直接读取原文。
- [iFLYTEK-OP/websdk-python](https://github.com/iFLYTEK-OP/websdk-python)：官方 Python SDK，Apache-2.0。
- [SDK 语音能力说明](https://github.com/iFLYTEK-OP/websdk-python/blob/master/websdk-python-speech/README.md)：含 `IatClient`、`TTSClient` 等接口。

官方协议明确：

- 在线听写地址为 `wss://iat-api.xfyun.cn/v2/iat`。
- 音频支持 16 kHz 或 8 kHz、16 bit、单声道；可选择 raw PCM，单次音频最长 60 秒。
- 音频状态为首帧 `0`、中间帧 `1`、末帧 `2`。
- 开启 `dwa=wpgs` 后，结果包含追加 `apd` 与替换 `rpl` 语义，需要按片段与替换范围合并。

### 5.2 与 Melodic 的适配设计

官方 SDK README 明确要求 **Python 3.7+**。Melodic 默认 Python 2，Ubuntu 18.04 通常自带的 Python 3.6 也不满足该要求。

建议采用独立的 Python 3.7+ 语音进程，使用隔离环境运行官方 SDK；由 Melodic 适配节点接收最终文本并发布任务请求。具体进程间接口留到实现阶段确定，不更改 ROS 所用的系统 Python。

语音层至少需要完成：麦克风录音、在线会话、动态修正合并、最终文本输出、错误反馈。只有完整最终指令通过解析与任务确认后才提交任务，避免流式中间结果触发重复运动。APPID、APIKey、APISecret 在部署时通过独立配置提供，不写入公开源码。

当前未核实虚拟机的麦克风输入、讯飞应用权限与额度，尚未完成真实在线识别调用。

### 5.3 已找到但不作为主依赖的封装

| 候选 | 实际适配限制 |
| --- | --- |
| [6-robot/xfyun_waterplus](https://github.com/6-robot/xfyun_waterplus) | GPL-2.0，使用 `libmsc.so` 等 Linux SDK；README 的发行版/分支说明需要进一步厘清，不是当前 WSS WebAPI 即用替代 |
| [zghong/xfyun-webapi](https://github.com/zghong/xfyun-webapi) | Ubuntu 18.04 C++ WebSocket 示例，可参考接口组织；未发现明确开源许可证，不直接复制进交付代码 |
| [picaun/xf_pyros_iat](https://github.com/picaun/xf_pyros_iat) | ROS1 语音节点封装，但要求 Python 3，且未发现明确许可证，不是 Melodic/Python 2 即用包 |

已有本地 `Voice_test/src/learning_robot` 的用途也已核对：`listening.cpp`、`speech_recognizer.c` 等链接 `libmsc.so`，发布 `voiceWords`；`voice_cmd.py` 将关键词直接转换为 `cmd_vel`，持续重发上一次速度。可以保留其文本话题与关键词映射思路，在线 WebAPI、任务语义和自主导航仲裁需要补写。

## 6. 推荐的完整演示流程

以下为拟议任务，不代表已经确认具体物品和场地。可采用带视觉标记的小盒子、固定取货台 A 和送货台 B，利用上游房间场景完成可重复演示。

示例指令：“把取货台 A 的一号盒子送到送货台 B。”

```text
录音 → 讯飞在线听写 → 最终文本解析 → 任务确认
  → 定位就绪 → 导航到取货区 → 视觉定位 → 底盘精对位
  → IK 与碰撞规划 → 抓取并验证 → 收臂
  → 导航到送货区 → 放置并验证 → 返回待命点 → 完成反馈
```

### 6.1 自主流程与复用关系

| 阶段 | 复用基础 | 需要补齐的闭环逻辑 |
| --- | --- | --- |
| 接收任务 | 讯飞在线听写、已有关键词思路 | 限定语法提取物品、取货点、送货点；不完整时请求补充 |
| 定位与导航 | AMCL、地图、`move_base` action | 初始定位条件、目标参数化、action 成功/失败/取消判断 |
| 目标定位 | 上游 AR 标记流程、TF | 标记 ID 对应物体、数据时效、目标位姿变换 |
| 抓取与放置 | 机器人模型、MoveIt/KDL、控制器 | 预抓取、接近、夹爪、抬升、收臂及每阶段成功条件 |
| 运输 | 上游导航配置 | 收臂与载物状态检查，送达后的放置流程 |
| 全任务组织 | SMACH、已有抓放状态机 | 任务参数、超时、有界恢复、取消、状态反馈 |

### 6.2 机械臂运动学展示

应能清楚演示以下关系：

1. 从 URDF 和关节反馈计算 FK，展示当前末端位姿。
2. 将视觉目标按其时间戳通过 TF 转到机械臂规划坐标系，生成可达的预抓取目标。
3. 调用 IK，检查关节限位；对求解结果做 FK 回代，记录位置误差。四自由度平台对可实现的姿态约束单独检查。
4. 将桌面、障碍物和目标物加入 MoveIt 场景，规划并执行轨迹，检查执行结果。
5. 独立验证夹爪动作和 Gazebo 物体状态；规划成功与抓取成功分别记录。

可参考 [MoveIt Melodic 运动学配置官方教程](https://github.com/moveit/moveit_tutorials/blob/melodic-devel/doc/kinematics_configuration/kinematics_configuration_tutorial.rst)。已读取该教程，确认 KDL、关节限位和 `position_only_ik` 配置含义。

### 6.3 TF、控制权与失败处理

- 导航使用一致的 `map → odom → base_link` 关系，机械臂和相机通过机器人模型连接。实际 frame 名称按所选平台及命名空间确定，避免重复发布同一 TF。
- Gazebo 使用仿真时钟；感知和操作检查时间戳，目标过期时重新获取。
- 导航、精对位、人工控制分阶段获得底盘控制权。上游导航和抓取节点都可能发布速度，需要统一切换或仲裁。
- 导航未成功不进入抓取；标记丢失时停止精对位并重新检测；IK 失败不发送无效轨迹；重试次数和时限均设上限。
- “取消任务”应传递到当前 action，并按是否载物、机械臂当前状态安全结束；取消成功后明确反馈。
- 只有抓取与放置条件成立才进入下一阶段，避免用固定等待时间替代结果判断。

可选加分点是 **视觉/TF 误差驱动的 PID 精对位**：在取货台附近比较基准控制与 PID 的收敛时间、稳态误差、超调量，并加入限幅与积分抗饱和。具体自由度取决于底盘：TurtleBot3 为差速底盘，不能直接执行横向速度；wpb_mani 支持全向运动。该项是后续候选改进，本轮未实现。

## 7. 实施顺序与验收依据

建议分三个里程碑完成，确保每一步都有可展示结果：

1. 同一机器人、同一 Gazebo 场景下完成地图定位、单点导航及 FK/IK 控制。
2. 使用固定文本任务完成“取货 → 搬运 → 放置”，并演示导航失败或目标丢失时的退出。
3. 接入讯飞真实麦克风输入，完成任务确认、执行、取消与状态反馈，再增加 PID 精对位对比。

每次演示至少记录：任务总耗时、导航 action 结果、目标定位与 IK 结果、抓取/运输/放置是否成功；汇总多次尝试的成功率。位置容差、超时与重试上限需根据所选场景实测确定，本报告不虚构数值。

建议展示四个场景：正常取送、未知物体/地点、导航或感知失败、执行中取消。相应的状态机和结果反馈比堆叠未经验证的功能更有利于说明系统完整性。

## 8. 其他线索的适用边界

- [Inventory-Management-AGV](https://github.com/sairampolina/Inventory-Management-AGV)：README 明确 Melodic，Apache-2.0；已读导航抓放状态机与 MoveIt 操作代码。可借鉴仓储任务组织，但依赖 TIAGo 仿真体系，操作位姿硬编码，抓取通过 `/setObjectState` 协作，物理抓持方式还需进一步核实。
- [TIAGo tutorials / melodic-devel](https://github.com/pal-robotics/tiago_tutorials/tree/melodic-devel)：确有 FK、IK 和抓放示例，可作运动学学习材料；不能把它当成独立完整的仿真安装包。
- [bt_fsm_comparison](https://github.com/ethz-asl/bt_fsm_comparison)：README 仅说明在 Noetic/Ubuntu 20.04 测试，依赖较多；适合作为行为树与状态机组织方式的参考。
- [wpr_simulation](https://github.com/6-robot/wpr_simulation)：当前 README 面向 Noetic/Ubuntu 20.04，不能仅因目录存在 Melodic 脚本就视为当前 Melodic 已验证。
- [executive_smach](https://github.com/ros/executive_smach)：任务执行框架。部署时优先使用 Melodic 对应的 `smach`、`smach_ros` 发行包，不要求仓库存在名为 `melodic-devel` 的分支。

本轮未把只看到搜索标题、未阅读正文的论坛帖子或 issue 当作技术结论依据。

## 9. 本轮交付与未验证项

本轮交付为开源选型、原始证据链接和拟议集成流程。保留现有 task1/task2/task3 和用户课程 README。

尝试执行一次本地文档检查，但自动审查将 PowerShell 语法判为无法解析的命令替换并阻止执行。因此 UTF-8/LF、围栏及链接格式检查未记为通过；没有进行 ROS 运行验证。

后续部署仍需确认：虚拟机图形与资源条件、所选基座依赖是否能在当前系统安装、Gazebo 抓持效果、视觉标记检测、讯飞账号及录音输入。选定具体物体、桌面高度和目标点后再填写任务配置，不从示例目录名称推测。
