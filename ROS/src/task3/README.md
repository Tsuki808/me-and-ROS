# task3：动态 TF2 与双通道 PID 小乌龟跟随

本设计依据课程的 turtlesim 跟随任务，以及用户确认的控制目标：通过 `/spawn` 创建独立的 `turtle2`，键盘控制 `turtle1`，第二只乌龟追踪第一只的当前位置；主龟停止后，两者位置接近重合。这里的复制通过创建第二个实例完成，两只乌龟分别接收运动指令。

## 1. 节点、话题和动态坐标系

| 节点/脚本 | 输入 | 输出/职责 |
| --- | --- | --- |
| `turtlesim_node` | 两只乌龟各自的 `cmd_vel` | 仿真运动，发布各自的 `pose`，提供 `/spawn` |
| `spawn_turtle.py` | `/spawn` 服务 | 在 `(1, 1, 3.14)` 创建 `turtle2` |
| `turtle_tf_broadcaster.py` × 2 | `/turtle1/pose`、`/turtle2/pose` | 持续广播 `world → turtle1` 和 `world → turtle2` |
| `turtle_teleop_key` | 终端方向键 | `/turtle1/cmd_vel` |
| `turtle_follower.py` | TF2 中 `turtle1` 相对 `turtle2` 的变换 | `/turtle2/cmd_vel` |

`world` 固定，两个乌龟坐标系的位置和朝向随运动变化。广播器使用 `TransformBroadcaster` 并更新时间戳，因此这些变换属于动态 TF，不需要再创建额外的移动目标坐标系。

跟随器调用：

```python
buffer.lookup_transform("turtle2", "turtle1", rospy.Time(0))
```

目标坐标系为 `turtle2`，源坐标系为 `turtle1`。其结果将主龟原点表示在从龟的局部坐标系中：

```text
T_turtle2_turtle1 = inverse(T_world_turtle2) * T_world_turtle1
distance_error = hypot(x, y)
heading_error  = atan2(y, x)            # [-pi, pi]
```

这样，角度误差相对从龟自身朝向计算，能够处理两只乌龟同时平移、旋转的情况。控制目标是位置接近；到达后不要求两只乌龟朝向一致。

## 2. PID 控制律与离散实现

turtlesim 的平面运动满足 `x_dot=v*cos(theta)`、`y_dot=v*sin(theta)`、`theta_dot=omega`。两通道分别根据距离误差和方位误差产生线速度、角速度，线速度约束随方位误差变化，因此两个通道在运动约束上相互关联。

每个有效采样周期使用 ROS 时间的实际差值 `dt`：

```text
delta_error = error[k] - error[k-1]
# 角度通道额外执行 wrap(delta_error) = atan2(sin(delta_error), cos(delta_error))
raw_D = delta_error / dt
alpha = dt / (derivative_tau + dt)
D[k] = D[k-1] + alpha * (raw_D - D[k-1])

I_candidate = clip(I[k-1] + error[k] * dt, -integral_limit, integral_limit)
u_candidate = Kp * error[k] + Ki * I_candidate + Kd * D[k]
```

第一帧建立时钟基准，PID 第一帧微分取零。角度微分先处理 `+pi/-pi` 跨界，避免把很小的实际角度变化计算成接近 `2*pi` 的跳变；一阶低通滤波降低微分对采样抖动的敏感度。

仅在 `abs(error) <= integral_zone` 时允许积分，误差超出范围则清零积分。若候选输出越过上限且误差为正，或越过下限且误差为负，拒绝本次积分增量，使用原积分重新计算限幅输出。这属于条件积分防饱和；允许使饱和程度减轻的反向积分增量。

最终指令为：

```text
linear_upper = max_linear_speed * max(0, cos(heading_error))
v     = clip(PID_distance, 0, linear_upper)
omega = clip(PID_heading, -max_angular_speed, max_angular_speed)
```

距离通道的防饱和直接使用 `linear_upper`，而非在防饱和之后再缩放指令。目标位于从龟后半平面时线速度为零，并清空距离 PID，只转向；该策略不让从龟倒车。角度通道仍可工作。

距离误差非负，单纯积分容易在到达后保留推进量。因此本实现配合积分分离与到达停车：距离不超过 `stop_distance` 时，发送全零速度并清空两个 PID；进入停车状态后，距离达到 `resume_distance` 才恢复跟随。两个阈值之间形成滞回，减少目标附近的反复启停。

## 3. 参数与故障处理

参数位于 `config/pid.yaml`，由 launch 加载到 `/turtle_follower` 的私有参数空间，修改后重启 launch 生效。

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `control_rate` | `20.0` | 控制频率，Hz；积分和微分仍使用实际 `dt` |
| `max_linear_speed` | `3.0` | 最大线速度，turtlesim 坐标单位/秒 |
| `max_angular_speed` | `4.0` | 最大角速度，rad/s |
| `stop_distance` / `resume_distance` | `0.10` / `0.15` | 停车/恢复距离，turtlesim 坐标单位 |
| `tf_timeout` | `0.50` | 允许的最大 TF 数据年龄，秒 |
| `max_dt` | `0.50` | 允许用于 PID 的最大采样间隔，秒 |
| `linear_pid: kp, ki, kd` | `1.20, 0.15, 0.08` | 距离通道参数 |
| `angular_pid: kp, ki, kd` | `4.00, 0.10, 0.15` | 方位通道参数 |
| 两通道 `integral_limit` | `0.50` | 积分状态 `integral(error*dt)` 的绝对值上限，非最终 I 项上限 |
| 距离/方位 `integral_zone` | `1.00` / `0.50` | 允许积分的误差范围，分别使用距离单位和 rad |
| 两通道 `derivative_tau` | `0.10` | 微分低通时间常数，秒 |

默认参数是该仿真的起始配置；参数的物理量纲随距离/角度通道不同。turtlesim 坐标单位没有经过实际机器人米制标定。节点检查参数是否有限、增益是否非负，以及 `resume_distance > stop_distance`、`max_dt > 1/control_rate`，错误配置会使 launch 结束。

TF 缺失、查询异常、时间戳在未来或年龄超过 `tf_timeout`、位置数据非有限时，节点持续发送零速度并清空 PID；TF 恢复后重新建立时间基准。`dt <= 0` 或大于 `max_dt` 时，该周期也停车并重置 PID，避免跨时间跳变或长暂停计算微分。正常关闭节点时发布零速度，跟随器和仿真器均设为 launch 的必要节点。

这些措施适用于正常进程与 ROS 通信。它们不构成真实机器人急停系统；跟随效果也取决于目标速度、转向、延迟与限幅，不能保证任意运动目标下误差始终为零。

## 4. 启动与键盘映射

虚拟机图形桌面终端一：

```bash
cd /home/jiayuexie/ROS
source devel/setup.bash
roslaunch task3 follow.launch
```

终端二，保持键盘焦点在该终端：

```bash
source /home/jiayuexie/ROS/devel/setup.bash
rosrun turtlesim turtle_teleop_key turtle1/cmd_vel:=/turtle1/cmd_vel
```

| 方向键 | 主龟运动 |
| --- | --- |
| 上 / 下 | 沿自身朝向前进 / 后退 |
| 左 / 右 | 逆时针 / 顺时针转向 |

显式话题重映射将键盘指令指向 `/turtle1/cmd_vel`。从龟仅接收 PID 输出。键盘节点需要真实终端输入，因此独立启动。按 `Ctrl+C` 关闭主 launch 结束仿真。

可在第三个终端观察：

```bash
rosparam get /turtle_follower
rostopic echo /tf
rostopic echo /turtle2/cmd_vel
```

## 5. 课程展示与验收

展示链路为“键盘主龟运动 → 两个动态 TF → 局部距离/方位误差 → 双通道 PID → 从龟速度”。相对原来的 `v=0.5*distance`、`omega=4*heading` 比例控制，课程改进点是完整的 I/D 状态、微分滤波、条件积分防饱和、朝向相关的速度限幅、滞回停车和 TF 过期停车。PID 是成熟方法，这些是本任务的控制实现改进。

人工验收包括：两只乌龟出现；方向键只直接控制主龟；从龟能从目标位于身后的情况转向并追踪；主龟停止后距离进入 `0.10` 附近并保持停车；主龟重新离开超过 `0.15` 后恢复跟随。

若需论证 PID 相对 P 的性能收益，应在相同初始位姿、目标运动和速度限幅下比较到达时间、跟踪误差与指令波动。将两个通道的 `ki`、`kd` 都设为 `0` 可得到保留同样保护逻辑的 P 对照。参数修改后须重启；没有对照数据时，不声称 PID 在所有指标上更优。

本次同步后的实际检查结果记录在 `VALIDATION.md`。
