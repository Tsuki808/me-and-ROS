# task3 PID 修改验证记录

日期：2026-10-15。通过已授权 SSH 在 Ubuntu 18.04.6、ROS Melodic、Python 2.7.17 上完成，工作区为 `/home/jiayuexie/ROS`，图形会话为 `DISPLAY=:0`。

本次仅执行一次汇总检查：统一构建后，运行一个无额外测试框架的简短脚本，依次检查文本与语法、PID 数值边界和仿真行为。整个命令退出码为 `0`。

| 检查项 | 实际结果 |
| --- | --- |
| UTF-8、LF、XML 和 Python 2 语法 | task3 源码、配置、launch 与文档通过 |
| catkin 构建 | `catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python2` 成功，三个包仍共用同一工作区 |
| 持续饱和时的积分 | 连续 10 次施加正误差并压低输出上限，指令保持限幅且积分状态未累积 |
| 角度跨越 `+pi/-pi` | 两个邻近角度样本跨边界时，滤波微分没有出现 `2*pi/dt` 型尖峰 |
| 初始追踪与到达停车 | 使用原始出生位姿，距离由 `6.427` 降至 `0.097`，随后线/角速度均为零 |
| 移动目标后重新到达 | 向主龟话题发布线速度 `1.0`、角速度 `0.8`，持续约 2 秒后发送零速度；从龟最终距离 `0.098` 并停车 |
| 从龟 TF 广播中断 | 在从龟已恢复运动后停止 `/turtle2_tf`；TF 年龄达到约 `0.513` 秒时出现过期提示，随后持续发布零速度 |
| 指令有效性与限幅 | 检查期间接收 208 条从龟速度指令，均为有限值，`0 <= linear.x <= 3.0`、`abs(angular.z) <= 4.0` |
| 结束处理 | 检查脚本正常退出并关闭本次启动的节点、turtlesim 和 ROS Master |

原始构建与运行日志：`/home/jiayuexie/ROS/logs/task3_pid_validation_20261015.log`。Windows 副本位于 `F:\University\Course\dasanshang\ROS_class\ROS\logs\task3_pid_validation_20261015.log`。本次检查脚本保存在虚拟机 `logs/check_task3_pid.py`，没有加入课程功能包或额外测试框架。

同步前的 task3 与虚拟机根 README 已备份至 `/home/jiayuexie/ROS/logs/task3_before_pid_20261015`。本地与虚拟机根 README 仅更新任务 3 相关内容，各自其余内容予以保留。

物理键盘输入、长期连续运行、TF 恢复后的重新跟随、时钟跳变及错误参数等情形未在本次仿真中逐项测试。移动目标检查通过向键盘使用的同一话题发布指令完成，不代替人工键盘操作。没有执行同条件 P/PID 对照实验，因此不据此宣称 PID 比原版本在所有性能指标上更优。原比例控制器的历史运行结果也不作为本 PID 版本的证据。
