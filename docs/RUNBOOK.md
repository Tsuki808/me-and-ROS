# ROS1 课程通用工作区



### 1. 每个新终端加载环境

下文每次要求新开终端时，先在 **`ROS` 根目录**执行：

```bash
export LANG=C.UTF-8
export LC_ALL=C.UTF-8
export PYTHONIOENCODING=utf-8
source /opt/ros/melodic/setup.bash
source devel/setup.bash
```



## 2. task1：ROS Hello World

```bash
roslaunch task1 hello.launch
```

节点初始化后输出一次 `Hello ROS!`，然后正常退出；这也是本任务的核心验收结果。实现沿用参考项目中 `rospy.init_node` 与 `rospy.loginfo` 的最小流程。

## 3. task2：自定义话题与学号查询服务


###  启动与验收

在第一个终端启动发布节点、订阅节点和服务端：

```bash
roslaunch task2 communication.launch
```


新开终端并加载环境，查看接口和消息：

```bash
rosmsg show task2/Student
rossrv show task2/QueryStudent
rostopic echo -n 1 /task2/student_info
rostopic hz /task2/student_info
```

观察约十秒后，频率应接近 `1 Hz`；按 `Ctrl+C` 停止频率观察。确认四个字段完整，学号仍为 `00000001`。

继续调用客户端，分别查询已知学号和未知学号：

```bash
rosrun task2 query_student_client.py 00000001
rosrun task2 query_student_client.py 99999999
```

预期第一条输出 `found=true` 及完整资料，第二条输出 `found=false`。客户端将位置参数直接作为字符串，保留前导零；服务尚未启动时，客户端等待服务可用。

也可以直接检查服务响应：

```bash
rosservice call /task2/query_student "student_id: '00000001'"
rosservice call /task2/query_student "student_id: '99999999'"
```




## 4. task3：动态 TF2 与 PID 小乌龟跟随



在虚拟机图形桌面的第一个终端运行：

```bash
source /home/jiayuexie/ROS/devel/setup.bash
roslaunch task3 follow.launch
```



第二个终端运行，并将键盘焦点留在该终端：

```bash
source /home/jiayuexie/ROS/devel/setup.bash
rosrun turtlesim turtle_teleop_key turtle1/cmd_vel:=/turtle1/cmd_vel
```

## 5. final project：小车机械臂

以下命令均在 Ubuntu 虚拟机的 `/home/jiayuexie/ROS` 目录运行。模式2启动时自动结束当前用户的旧ROS后台和仿真，再启动新一轮；模式1、3的启动方式保持原样，切换前先按 `Ctrl+C` 结束旧仿真。三种模式均默认打开 Gazebo、RViz 和全部传感器。

### 模式1：键盘控制

终端1，启动仿真：

```bash
source /home/jiayuexie/ROS/devel/setup.bash
roslaunch final_project simulation.launch mode:=1
```

终端2，启动键盘控制，并将键盘焦点留在此终端：

```bash
source /home/jiayuexie/ROS/devel/setup.bash
rosrun final_project keyboard_control.py
```

按键：

- R：收臂准备，允许驾驶。
- W/S/A/D：前后、左右转；T：细调速度。
- 1/2：选择 A／B 台。
- P/O：执行完整抓取／放置。
- 空格：停车并取消当前动作。
- F：返航后确认任务完成；Q：退出。

### 模式2.1：全自动建图与自主取送



**构建与启动**

以下命令在 Ubuntu 虚拟机执行。源码更新后先构建；已完成构建时，直接加载工作区并启动即可。自动清理在最后一条roslaunch命令内部执行，无需额外手动清理命令。

```bash
cd /home/jiayuexie/ROS
source /opt/ros/melodic/setup.bash
chmod +x src/final_project/scripts/*.py src/final_project/tools/render_robot.py
catkin_make -j2 -l2
source /home/jiayuexie/ROS/devel/setup.bash
roslaunch final_project simulation.launch mode:=2 target_color:=red
```

**观察、取消与结果**

可在独立终端查看流程阶段或探索状态；每条 `echo` 持续监听，按 `Ctrl+C` 退出后可执行下一条：

```bash
source /home/jiayuexie/ROS/devel/setup.bash
rostopic echo /final_project/status
rostopic echo /final_project/exploration/status
```

需要取消整个模式2流程时，在另一个终端执行：

```bash
source /home/jiayuexie/ROS/devel/setup.bash
rosservice call /final_project/mode2/cancel '{}'
```

取消或失败后保持停车，已抓物体保留附着，不自动松爪或恢复。重新执行模式2启动命令会清理旧现场并开始新任务。若需保留已夹持盒子继续放置，使用项目README中的显式持箱续跑action，不重新执行simulation.launch。成功后保留最终地图显示，按启动终端的 `Ctrl+C` 关闭。

### 模式2.2：视觉语言模型驱动的语义取物与可解释技能执行

模式2.2是独立的桌边演示入口，不改变模式2.1、模式1和模式3。机器人从 A 台前启动，Windows 操作台接收当前 YOLO 观察帧、中文文字和可选参考图；Qwen3-VL 只负责在本帧候选编号中选择目标，ROS 端再次核对候选颜色、ALVAR 标记和模型身份，然后复用原视觉 PID、抓取约束、举示 2 秒、原位放回和退离停车。

Windows 使用已下载的 `Qwen3-VL-4B-Instruct-GGUF` 两个 GGUF 文件，通过 CUDA 版 `llama-server` 提供 OpenAI 兼容接口；ROS 虚拟机使用 Melodic、rosbridge 和原有 YOLO/MoveIt/Gazebo。模型不生成速度、关节角或物理坐标。观察帧包含 `observation_id`、同帧 JPEG 和 YOLO 候选；动作目标必须引用仍然有效的观察帧，窗口心跳中断 3 秒会停车并取消。

虚拟机启动（先完成 `rosbridge_server` 安装并重新构建）：

```bash
cd /home/jiayuexie/ROS
source /opt/ros/melodic/setup.bash
chmod +x src/final_project/scripts/*.py src/final_project/tools/render_robot.py
catkin_make -j2 -l2
source /home/jiayuexie/ROS/devel/setup.bash
roslaunch final_project mode22.launch
```

Windows 启动本地模型服务后运行操作台：

```powershell
Start-Process -FilePath 'E:\Project\llama.cpp-b11836-win-cuda12\bin\llama-server.exe' -ArgumentList @('-m', 'E:\Project\models\Qwen3-VL-4B-Instruct-GGUF\Qwen3VL-4B-Instruct-Q4_K_M.gguf', '--mmproj', 'E:\Project\models\Qwen3-VL-4B-Instruct-GGUF\mmproj-Qwen3VL-4B-Instruct-F16.gguf', '-c', '4096', '-np', '1', '--image-min-tokens', '1024', '-ngl', '99', '--host', '127.0.0.1', '--port', '8080')
& 'E:\Programming\.conda\envs\mode22\python.exe' 'F:\University\Course\dasanshang\ROS_class\ROS\src\final_project\embodied\app.py' --ros-host 192.168.162.131 --server-port 7860
```

界面中的“分析并执行”只接受完整模型决策；目标不存在、候选过期、模型格式错误或 ROS 未连接时保持停车。取消按钮使用 `/final_project/embodied/cancel`，动作反馈位于 `/final_project/status` 和 `SemanticPick` action。当前代码检查已覆盖 Python 语法、XML 解析和 action/launch 引用；Windows 模型服务、rosbridge 安装及虚拟机端到端动作尚未在本次静态修改后验证。


### 模式3：语音控制

终端1，启动仿真及语音控制：

```bash
source /home/jiayuexie/ROS/devel/setup.bash
roslaunch final_project simulation.launch mode:=3
```

等待 `READY MODE3` 和 `VOICE_LISTENING`，先说“小车准备”，收臂成功后再发出其他指令。每句话以“小车”开头，每次只说一条完整指令：

- 小车前进／后退／左转／右转：连续运动；10秒无新方向指令自动停车。
- 小车停止／取消：停车并取消当前动作。
- 小车普通／细调：停车并选择速度档。
- 小车微调前进／后退／左转／右转：低速点动0.25秒后停车。
- 小车选择A台／选择B台：选择抓放台，不自动导航。
- 小车抓取／放置：对准后执行原完整抓取／放置动作。
- 小车完成：自行返航后检查任务完成，不自动返航。
- 小车状态：显示当前状态；小车退出：关闭语音控制，保留仿真。

可选终端2，查看语音接收反馈与对位提示：

```bash
source /home/jiayuexie/ROS/devel/setup.bash
rostopic echo /final_project/voice_status
```

讯飞独立环境及账号已配置；真实麦克风连续识别及“状态→准备→微调前进→停止”已通过。完整语音搬运与运动中语音停车尚未验收。模式3沿用模式1的手动行驶与对位要求，云端识别存在延迟，精对位使用微调指令。详细结果见项目 `VALIDATION.md`。
