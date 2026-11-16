<div align="center">

# Me and ROS

**ROS 通信、TF2、PID 这些基础，自动建图取送，以及用视觉语言模型取物**

<sub>《Linux 基础及 ROS 应用实践》课程项目 · ROS 1 Melodic · Gazebo 9 · TurtleBot3 Waffle + OpenMANIPULATOR</sub>

<br>

![ROS](https://img.shields.io/badge/ROS-Melodic-22314E?style=flat-square)
![Ubuntu](https://img.shields.io/badge/Ubuntu-18.04-E95420?style=flat-square)
![Python](https://img.shields.io/badge/Python-2.7-3776AB?style=flat-square)
![Gazebo](https://img.shields.io/badge/Gazebo-9-FF6600?style=flat-square)
![MoveIt](https://img.shields.io/badge/MoveIt-1.0.11-0A66C2?style=flat-square)
![VLM](https://img.shields.io/badge/Qwen3--VL-4B-6A5ACD?style=flat-square)
![AUV](https://img.shields.io/badge/AUV-RoboCup%202026-0B7285?style=flat-square)

</div>

---

## 现场演示

<sub>下面的动图**打开就自己循环播放，不用点**。画面来自真实运行录像，做了 2–5 倍加速和压缩。</sub>

<table>
<tr>
<td width="50%" align="center">
<img src="docs/media/preview/preview_m21_mapping.gif" width="100%" alt="模式2.1 自动建图">
<br><sub><b>模式 2.1 · 自动建图</b><br>GMapping + frontier 探索，地图从空白长到闭合</sub>
</td>
<td width="50%" align="center">
<img src="docs/media/preview/preview_m21_pick.gif" width="100%" alt="模式2.1 A台抓取">
<br><sub><b>模式 2.1 · A 台抓取</b><br>视觉对位 → 夹爪闭合 → 离台抬升</sub>
</td>
</tr>
<tr>
<td width="50%" align="center">
<img src="docs/media/preview/preview_m21_place.gif" width="100%" alt="模式2.1 B台放置">
<br><sub><b>模式 2.1 · B 台放置</b><br>对位 → 下降 → 释放 → 退离</sub>
</td>
<td width="50%" align="center">
<img src="docs/media/preview/preview_m22_semantic.gif" width="100%" alt="模式2.2 语义取物">
<br><sub><b>模式 2.2 · 语义取物</b><br>Qwen3-VL 选中 obj_1 → 抓取 → 举示 → 放回</sub>
</td>
</tr>
</table>

<p align="center">
<img src="docs/media/preview/preview_auv.gif" width="340" alt="AUV 入水与下潜">
<br><sub><b>AUV · 入水与下潜</b>　RoboCup 水下机器人赛项现场</sub>
</p>

<p align="center"><sub>完整版 MP4（2–5 倍加速）放在
<a href="#二模式21全自动建图与自主取送">模式 2.1</a>、
<a href="#三模式22视觉语言模型驱动的语义取物与可解释技能执行">模式 2.2</a> 和
<a href="#五auv-进阶实践成果佐证">AUV</a> 章节里；上面这些只是让人一打开就看到动作。</sub></p>

---

## 目录

| 章节 | 内容 | 关键产出 |
| :--- | :--- | :--- |
| [现场演示](#现场演示) | 首屏自动循环片段 | 5 段 GIF（建图 / 抓取 / 放置 / 语义 / AUV） |
| [一](#一项目概览) | 项目概览与环境 | 四个入口、一套底层 |
| [二](#二模式21全自动建图与自主取送) | **模式 2.1：全自动建图与自主取送** | 382.9 s 自主探索、A→B 抓放、返航 |
| [三](#三模式22视觉语言模型驱动的语义取物与可解释技能执行) | **模式 2.2：VLM 语义取物与可解释技能执行** | Qwen3-VL 受限决策 + ROS 六项校验 |
| [四](#四基础任务与其余入口) | task1 / task2 / task3 / 模式1 / 模式3 | 通信、TF2+PID、键盘、语音 |
| [五](#五auv-进阶实践成果佐证) | AUV 进阶实践（仅成果佐证） | RoboCup 水下机器人赛项 |
| [六](#六复现与验证) | 复现步骤与验证记录 | `VALIDATION.md`、`RUNBOOK.md` |
| [七](#七仓库说明) | 收录范围、许可与致谢 | 源码边界声明 |
| [媒体索引](docs/MEDIA_INDEX.md) | 截图与视频的原始文件名对照、视频处理参数 | 85 张截图 · 5 段 GIF · 4 段视频 |

---

## 一、项目概览

这个仓库记的是一门 ROS 课从基础到项目的过程：先自己写节点，弄明白话题、服务、TF2 和 PID，再用同一套底盘和机械臂做出**四种控制方式**——键盘、语音、全自动、视觉语言模型。四种方式共用同一份 URDF、传感器、TF 树、MoveIt 场景、夹爪插件和速度门控，区别只在于"**谁来定目标、什么时候动手**"。

| 入口 | 决策主体 | 底层技能 | 一句话说明 |
| :--- | :--- | :--- | :--- |
| 模式 1 | 人（按键） | ManualAction、速度门控 | R 收臂解锁，W/S/A/D 驾驶，P/O 抓放 |
| **模式 2.1** | **程序**（地图 + frontier + YOLO 候选） | GMapping / AMCL / move_base / MoveIt / 视觉 PID | **一键：建图 → 定位 → A 抓 → B 放 → 返航** |
| **模式 2.2** | **模型**（在候选编号中选一个） | 复用 2.1 的全部抓放技能 | **说一句话，机器人在同帧候选里挑目标并执行** |
| 模式 3 | 人（闭集语音） | ManualAction、速度门控 | "小车前进"等有限指令，经同一安全出口 |

<details>
<summary><b>实验环境（点击展开）</b></summary>

| 项目 | 实际配置 | 说明 |
| :--- | :--- | :--- |
| 操作系统 | Ubuntu 18.04.6 LTS，内核 5.4.0-150 | VMware 虚拟机 |
| ROS | Melodic，roscore 1.14.13 | Python 2.7.17 / rospy |
| 仿真 | Gazebo 9.0.0 | ODE 物理引擎 |
| 机器人 | TurtleBot3 Waffle + OpenMANIPULATOR | MoveIt 1.0.11 |
| 工作空间 | `/home/jiayuexie/ROS` | task1、task2、task3、final_project |
| 构建 | `catkin_make -j2 -l2` | 4 GB 内存虚拟机限流 |
| 语义模型 | Qwen3-VL-4B-Instruct-GGUF（Q4_K_M + F16 mmproj） | Windows 侧 llama.cpp CUDA |

版本不是装饰：Melodic 对应 Python 2.7，Gazebo 9 的差速插件与 MoveIt 1.0.11 的接口与 Noetic 不同，后文所有命令与数值都以这套版本为准。

</details>

### 仓库结构

```text
me-and-ROS/
├── ROS/src/
│   ├── task1/                  # Hello ROS
│   ├── task2/                  # 自定义消息、话题、学号查询服务
│   ├── task3/                  # 动态 TF2 + 双通道 PID 小乌龟跟随
│   └── final_project/          # 核心工程：四入口 + 建图取送 + 语义取物
│       ├── scripts/            # mission_server、automatic_delivery、semantic_mission…
│       ├── src/                # 探索节点、条件抓取插件、控制库
│       ├── launch/             # simulation / mode22 / navigation / manipulation
│       ├── embodied/app.py     # 模式2.2 Windows 操作台
│       ├── vision/             # YOLOE-26n-seg ONNX 与导出脚本
│       ├── voice/              # 讯飞流式听写接入
│       ├── config/ urdf/ worlds/ maps/ models/
│       ├── README.md           # 工程级详细说明
│       └── VALIDATION.md       # 逐轮部署与运行验证记录
├── docs/
│   ├── RUNBOOK.md              # 原始运行手册（逐条命令）
│   ├── MEDIA_INDEX.md          # 图片/视频原始文件名对照
│   ├── OPENSOURCE_SURVEY.md    # 导航/语音/机械臂方案调研
│   └── media/                  # 模式2.1、2.2、其余入口的截图与演示视频
└── AUV/                        # 比赛成果佐证（不含任何源代码）
```

---

## 二、模式 2.1：全自动建图与自主取送

> **敲一条命令，机器人从空白地图开始，自己建图、自己去 A 台抓、放到 B 台，最后回到待命点停下。** 中间不用点目标，不用选方向，也不用确认开始。

```bash
roslaunch final_project simulation.launch mode:=2 target_color:=red
```

### 完整流程

```mermaid
flowchart LR
    A["PREPARING<br/>收臂 · 等实时建图"] --> B["MAPPING<br/>GMapping + frontier 探索"]
    B --> C["SAVING_MAP<br/>冻结 · 逐格校验 PGM/YAML"]
    C --> D["LOCALIZING<br/>加载地图 · AMCL 初始化"]
    D --> E["DELIVERING<br/>A 抓 → B 放 → 返航"]
    E --> F["SUCCEEDED<br/>写 run.json · 停车"]
```

| 阶段 | 入口时刻 / s（仿真） | 阶段耗时 / s | 该阶段只做一件事 |
| :--- | ---: | ---: | :--- |
| `PREPARING` | 0.304 | 6.1 | 收回机械臂，等待实时建图可用 |
| `MAPPING` | 6.374 | 384.4 | 自动 frontier 探索，边探索边建图 |
| `SAVING_MAP` | 390.796 | 1.1 | 冻结地图快照并逐格核对 |
| `LOCALIZING` | 391.872 | 2.9 | 停建图、起 AMCL、静止校正 |
| `DELIVERING` | 394.820 | 248.9 | A 台抓取、B 台放置、返航 |
| `SUCCEEDED` | 643.748 | — | 终态验收并写盘 |

> 表中为仿真时间。交付段墙钟耗时 529.6 s，对应仿真实时率约 0.47——观测开销（截帧、录像）与物理仿真开销是分开的，验收动作不改变控制参数。

### 三个关键点

**① 建图探索：先找出一批候选位置**

frontier 就是已知空地和未知区域的分界线。程序没有直接拿分界线的中心当目标，而是隔一段取一个点，最多取 20 个；每个点周围按 3 个半径（0.35、0.55、0.80 m）和 16 个方向摆出候选观察位。一个候选要同时满足两个条件：`safe(p)`（落在已知空地上、容得下机器人）和 `visible(p,f)`（到 frontier 的连线不被障碍挡住），相距 0.12 m 以内的算重复，只留一个。排序用的代价是：

```text
S(p) = ‖p − r‖ + 0.5 · ‖p − f₀‖
       └─行驶成本─┘   └─离 frontier 过远的惩罚─┘
```

排好序的前 12 个候选还要逐个调用 `make_plan`，确认**当前地图上真的走得通**才交给 `move_base`。失败的点记在 0.35 m 的范围内，同一处 frontier 最多试两次；frontier 最小算 0.30 m，而且边界点之间最大的跨度也要 ≥ 0.30 m，免得一小块未知区域的弯曲边缘被反复挑中。

什么时候算建完，不是看时间到没到，而是看**有效的 frontier 用完了，或者只剩下小块封闭的未知区域**：跟剩余边界连着的未知区按四邻域搜一遍，每块 ≤ 0.20 m²、加起来 ≤ 0.30 m²，而且不能伸出搜索范围或地图边缘。这样"看着差不多"就有了一个能核对的面积标准。

**② 建图转定位：同一时刻只让一个节点发布 `map→odom`**

GMapping 在跑的时候会发布 `map→odom`，AMCL 起来后也发同一条变换；两个一起发，TF 树就会有多个父节点，导航和 RViz 都乱套。所以状态机把建图和定位**排成先后**：先停车、把地图定格、逐格核对 PGM/YAML，再停掉建图节点，最后加载地图、启动 AMCL。

AMCL 不是从头做全局定位，而是拿保存地图时机器人的位姿当起点，连着做 8 次静止更新，要求协方差对角项 ≤ 0.04、交接修正 ≤ 0.5 m / 0.5 rad。这次实测静止校正 0.666 s，最大位移 **0.000728 m**、最大航向漂移 **0.000136 rad**。

**③ 抓取和放置：三道检查，过不了就不动手**

| 层次 | 判据 | 失败行为 |
| :--- | :--- | :--- |
| 视觉对位 | 误差进入 \|e_d\| < 0.012 m、\|e_θ\| < 0.04 rad 后先发零速度，再连续稳定 0.25 s 才返回 | 继续对位；标记超 2.0 s 或丢帧超 3.0 s 则停车 |
| 夹爪反馈 | 闭合误差 ≤ 0.005 m、张开误差 ≤ 0.002 m、稳定 0.2 s | 抛 `Gripper feedback mismatch`，不调用抓取 |
| 几何包络 | 两指关节位置 ≤ 0.008 m **且**盒心与末端原点 3D 距离 ≤ 0.035 m | 拒绝附着，回报 `outside the 35 mm grasp envelope` |

抓取点同时参考 ALVAR 标记和 Gazebo 里的模型状态：一般以标记为准；两者差超过 0.02 m 就改用模型状态。放得对不对也不是"松手就算"，而是松手后 5 s 内反复读模型状态，平面误差 < 0.08 m、高度误差 < 0.025 m、线速度 < 0.03 m/s **同时满足**才记下 `placement_error`。

### 这次的结果

| 项目 | 本次结果 |
| :--- | :--- |
| 探索结束 | 仿真 382.878 s，`failures=0`，残余 0.25375 m²（0.1000 / 0.0844 / 0.0694 三块，均 < 0.20 m²） |
| 地图 | 352 × 672 栅格，分辨率 0.025 m，origin [-4.5, -3.5] |
| AMCL 静止检查 | 0.666 s，最大位移 0.000728 m，最大航向漂移 0.000136 rad |
| A 台对位 | −0.00716 m / 0.02788 rad（visual = true） |
| B 台对位 | 0.01172 m / 0.02089 rad（visual = false） |
| 抓取 | 成功，最大 FK 回代残差约 1.14 × 10⁻¹⁶ m |
| 放置误差 | 0.07858 m（容差 0.08 m，余量 1.42 mm） |
| 终态 | `HOLD`、`attached=False`、`cmd_vel` 全零；红盒在 B 台，机器人回到待命区 |
| 恢复动作 | 导航重试 1 次、导航/PID 交接 3 次，仍收敛到终态 |

<div align="center">
<img src="docs/media/mode2_1/14_final_acceptance.png" width="820" alt="模式2.1 终态验收">

<sub>终态验收终端：status / control_mode / attached / cmd_vel / 物体与机器人位姿 / 对位与放置误差</sub>
</div>

### 现场记录

**运行总览**

<div align="center">
<img src="docs/media/mode2_1/24_workspace_overview.png" width="820" alt="模式2.1 运行现场">
</div>

<p align="center"><sub>Gazebo 世界、RViz 地图与多传感器、YOLO 观察窗口、终端状态同屏</sub></p>

**建图：从空白到闭合**

<p align="center">
<img src="docs/media/mode2_1/03_mapping_mid.png" width="49%" alt="建图中期">
<img src="docs/media/mode2_1/05_mapping_late.png" width="49%" alt="建图后期">
</p>
<p align="center">
<img src="docs/media/mode2_1/23_final_map.png" width="46%" alt="最终地图">
<img src="docs/media/mode2_1/04_frontier_status.png" width="50%" alt="frontier 状态">
</p>

<p align="center"><sub>上排：建图中期与后期，墙体逐步闭合、激光束变长；下排：左为最终栅格地图，右为探索状态终端（frontier 递减、failures=0）</sub></p>

**抓取与放置**

<p align="center">
<img src="docs/media/mode2_1/07_yolo_four_boxes.png" width="49%" alt="A台四目标检测">
<img src="docs/media/mode2_1/09_pick_success.png" width="49%" alt="抓取成功">
</p>
<p align="center">
<img src="docs/media/mode2_1/11_b_place.png" width="49%" alt="B台放置">
<img src="docs/media/mode2_1/13_return_home.png" width="49%" alt="返航停车">
</p>

<p align="center"><sub>上：A 台同帧检出红/蓝/黄/绿四盒（目标色为红），红盒被夹持抬起；下：B 台释放、返航停车后的 RViz 地图</sub></p>

**动作序列联络表**（由 2 fps 录像抽帧合成，覆盖检测、对位、抓放、退离）

<p align="center">
<img src="docs/media/mode2_1/19_pick_sheet.jpg" width="49%" alt="A台动作联络表">
<img src="docs/media/mode2_1/20_place_sheet.jpg" width="49%" alt="B台动作联络表">
</p>

### 演示视频

上传前统一做了 **2 倍加速 + 截取核心片段**，用 H.264（CRF 28）重编码，单段压到 1 MB 左右。首屏的动图就是这些 MP4 的压缩预览。完整片段以链接形式放在下面：GitHub 的 README 会把 `<video>` 标签去掉，仓库里的 MP4 没法直接嵌在页面里播放，只能打开链接看。

<table>
<tr>
<td width="33%" align="center">
<a href="docs/media/video/mode2_1_pick.mp4"><img src="docs/media/video/mode2_1_pick_poster.jpg" alt="A台抓取"></a>
<br><sub><b>A 台抓取</b><br>59 s · 2× 加速<br>对位 → 夹取 → 抬升</sub>
</td>
<td width="33%" align="center">
<a href="docs/media/video/mode2_1_place.mp4"><img src="docs/media/video/mode2_1_place_poster.jpg" alt="B台放置"></a>
<br><sub><b>B 台放置</b><br>62 s · 2× 加速<br>对位 → 下降 → 释放 → 退离</sub>
</td>
<td width="33%" align="center">
<a href="docs/media/video/mode2_1_mapping.mp4"><img src="docs/media/video/mode2_1_mapping_poster.jpg" alt="建图延时"></a>
<br><sub><b>建图延时</b><br>7 s · 2× 加速<br>frontier 探索地图生长</sub>
</td>
</tr>
</table>

### 一次典型失败与修复

第一轮抓取只回了一句 `Object is outside the 35 mm grasp envelope`。是抓取插件拒绝了，原因在近距离上几个误差叠在一起：ALVAR 近距离斜着看，给出的标记点跟 Gazebo 模型点差了约 **3.3 cm**，再加上 **1.2 cm** 的抓取高度偏差，末端到盒心的三维距离刚好超过 35 mm。

处理办法没有放宽容差，而是加了**一致性校正**：视觉对位照旧，只在标记点和模型点差超过 2 cm 时，改用模型状态作为最终抓取点，并把触发条件和替换规则写进状态里。

```text
Grasp target corrected: marker=(0.268, 0.007) model=(0.267, -0.025) error=0.033 m
```

<p align="center">
<img src="docs/media/mode2_1/21_grasp_envelope_fail.png" width="49%" alt="35mm包络失败">
<img src="docs/media/mode2_1/22_model_correction.png" width="49%" alt="模型校正">
</p>

<p align="center"><sub>左：第一轮 35 mm 包络失败现场；右：模型一致性校正日志，随后 PICK / PLACE / RETURN_HOME 全部成功</sub></p>

### 复现

```bash
cd /home/jiayuexie/ROS
source /opt/ros/melodic/setup.bash
chmod +x src/final_project/scripts/*.py src/final_project/tools/render_robot.py
catkin_make -j2 -l2
source /home/jiayuexie/ROS/devel/setup.bash
roslaunch final_project simulation.launch mode:=2 target_color:=red
```

观察阶段与取消：

```bash
rostopic echo /final_project/status
rostopic echo /final_project/exploration/status
rosservice call /final_project/mode2/cancel '{}'
```

取消或失败后保持停车、已抓物体保留附着，不自动松爪；重新执行启动命令会清理旧现场并开始新任务。参数、话题与阶段细节见 [`final_project/README.md`](ROS/src/final_project/README.md)。

---

## 三、模式 2.2：视觉语言模型驱动的语义取物与可解释技能执行

> **模型只管"挑哪一个"，不动手。** Qwen3-VL 从当前画面里的候选编号中选一个，输出 `decision / candidate_id / summary`；坐标、速度、关节角它一概不给。ROS 这边再核对颜色、画面是不是太旧、置信度排序、物体实际位置和 ALVAR 标记，然后沿用模式 2.1 的视觉 PID、抓取限制、举示 2 s、放回原位和退离停车。

### 数据流

```mermaid
flowchart TB
    subgraph VM["ROS 虚拟机 · Melodic"]
        CAM["检查相机 + YOLO/HSV"] --> OBS["observation<br/>obs_id + 同帧 JPEG + 候选列表"]
        OBS --> BR["rosbridge"]
        SEM["semantic_mission<br/>六项确定性校验"] --> SKILL["既有技能<br/>视觉 PID → MoveIt → 夹爪 → 举示 → 放回 → 退离"]
        VG["速度门控<br/>唯一 cmd_vel 写者"] --> BASE["底盘"]
        SKILL --> VG
    end
    subgraph WIN["Windows 操作台"]
        UI["Gradio 控制台<br/>分析场景 / 确认并执行"] --> LLM["Qwen3-VL-4B<br/>llama-server CUDA"]
        LLM --> UI
    end
    BR --> UI
    UI -->|"SemanticPick goal"| SEM
```

观察话题 `/final_project/embodied/observation` 以 **2.6–2.8 Hz** 发布；每个观察帧包含 `observation_id`、同帧 JPEG 和 YOLO 候选（`candidate_id`、颜色、标签、置信度、bbox）。动作目标必须引用**仍然有效**的观察帧。

### 让模型只做选择题

| 约束项 | 取值 |
| :--- | :--- |
| 固定中文指令 | "拾取当前视野中 YOLO 检测置信度最高的目标盒子，完成举示动作后放回原位" |
| 模型输入 | 同帧带标注 JPEG + 候选列表 |
| 输出 schema | `decision ∈ {answer, pick_show_return, clarify}`、`candidate_id`、`summary` |
| 解码参数 | `temperature=0`、`max_tokens=256`、`response_format=json_schema` |
| 显式禁止 | 不生成坐标、速度、关节角 |

说白了就是让模型做一道选择题：画面里有候选集合 `C = {c₁ … cₙ}` 和这一帧图像 `I`，模型输出 `d* = argmax_{c∈C} s(I, c)` 和一个决策标签。控制台还做了防误触：没点"分析场景"就去点"确认并执行"，界面会回"请先分析当前场景"，**不会给 ROS 发任何 goal**；就算分析过了，执行前还要再确认候选仍在当前这一帧里、而且是置信度最高的那个。

### ROS 这边的六项检查

动手之前，`semantic_mission` 要把六项检查 `V = V_color ∧ V_age ∧ V_candidate ∧ V_argmax ∧ V_pose ∧ V_marker` 都过一遍，任何一项不满足就抛异常、Action 判失败、机器人保持停车：

| 校验 | 内容 | 失败含义 |
| :--- | :--- | :--- |
| `V_color` | 目标颜色必须在 `colored_boxes` 中 | 指令越界 |
| `V_age` | 在最近 256 条观察历史中找到该 `observation_id`，且距接收 ≤ 300 s | 用旧场景执行新命令 |
| `V_candidate` | `candidate_id` 存在且颜色与 goal 一致 | 候选串号 |
| `V_argmax` | 候选必须是该帧 YOLO 置信度最高者 | 模型选了非最优 |
| `V_pose` | 读取模型状态，确认盒子仍在 A 台期望位置 0.12 m 内 | 物体已被移动 |
| `V_marker` | 模型名与 ALVAR 标记绑定一致（黄 8 / 红 9 / 绿 10 / 蓝 11） | 物理身份不符 |

前四项只用观察历史和 goal 就能算，后两项才去问 Gazebo。检查从省事的先做，不过就**短路**——这么做不是为了快，是为了安全：先用不费力的条件把明显有问题的 goal 挡掉，再用实际状态确认"这东西现在确实还在 A 台"。

### 三道安全关卡

| 检查层 | 规则 | 失败结果 |
| :--- | :--- | :--- |
| Windows 控制台 | 必须先分析；候选仍存在；必须是最高置信度 | 拒绝执行，不发送 goal |
| ROS 语义节点 | observation ≤ 300 s；颜色一致；最高置信度；模型在 A 台 0.12 m 内 | Action 失败，保持停车 |
| 运行时 | Windows 心跳 ≤ 3 s；每个阶段检查取消 | `stop()`，零速度，取消机械臂/夹爪 |

心跳监测就放在取消检查里，超过 3 s 就停车；`SHOW` 阶段每 50 ms 查一次，所以"举示 2 s"并不是一个打断不了的过程。动作按顺序是：

```text
VALIDATE_TARGET → PREPARE → DOCK_PICK_PID → PICK → SHOW（2 s） → PLACE（原位） → RETREAT（0.15 m） → SUCCEEDED
```

### 这次的结果

| 项目 | 本次结果 |
| :--- | :--- |
| 观察帧 | `obs_835`；该观察位同帧检出红/蓝/黄/绿四个候选 |
| 模型决策 | `pick_show_return`，`candidate_id = obj_1`（黄），置信度 0.2562 |
| 模型耗时 | 2.982 s（本地 4B 量化模型，CUDA） |
| 动作耗时 | 110.31 s，`success=true` |
| 终态 | `SUCCEEDED: Semantic pick/show/return completed`，`attached=False`，`mode=HOLD`，`cmd_vel` 全零 |
| 放回精度 | 黄盒 (−1.2012, 0.9154, 0.2050)，与原始 (−1.20, 0.90, 0.205) 平面偏差约 **0.0154 m** |
| 退离 | 0.150 m，与配置一致 |

<p align="center">
<img src="docs/media/mode2_2/07_qwen_decision.png" width="49%" alt="Qwen 决策">
<img src="docs/media/mode2_2/08_observation_json.png" width="49%" alt="观察帧与结构化决策">
</p>

<p align="center"><sub>左：`decision=pick_show_return`、`candidate_id=obj_1`、置信度、推理耗时；右：同帧图像与结构化 JSON——没有任何坐标、速度或关节角</sub></p>

### 现场记录

**观察与拦截**

<p align="center">
<img src="docs/media/mode2_2/03_four_candidates.png" width="49%" alt="四候选">
<img src="docs/media/mode2_2/06_console_guard.png" width="49%" alt="防误触">
</p>

<p align="center"><sub>左：观察位同帧检出四个候选，"多候选择一"不是空话；右：未分析直接执行被拦截，阶段显示"未执行"</sub></p>

**执行链：对位 → 抓取 → 举示 → 放回 → 退离**

<p align="center">
<img src="docs/media/mode2_2/10_dock_before_pick.png" width="32%" alt="抓取前">
<img src="docs/media/mode2_2/11_pick_success.png" width="32%" alt="抓取成功">
<img src="docs/media/mode2_2/12_show_hold.png" width="32%" alt="举示2秒">
</p>
<p align="center">
<img src="docs/media/mode2_2/13_place_back.png" width="32%" alt="原位放回">
<img src="docs/media/mode2_2/14_detection_resume.png" width="32%" alt="检测恢复">
<img src="docs/media/mode2_2/22_final_rviz.png" width="32%" alt="终态 RViz">
</p>

<p align="center"><sub>上：夹爪对准黄盒、抓取离台、举示保持 2 s；下：原位放回、YOLO 恢复检测、退离后终态</sub></p>

**终态验收**

<div align="center">
<img src="docs/media/mode2_2/19_final_acceptance.png" width="800" alt="模式2.2 终态">
</div>

<p align="center"><sub>`SUCCEEDED` / `attached=False` / `HOLD` / 零速度，黄盒坐标与机器人退离位置同屏可核</sub></p>

### 演示视频

<table>
<tr>
<td width="60%" align="center">
<a href="docs/media/video/mode2_2_semantic_pick.mp4"><img src="docs/media/video/mode2_2_pick_poster.jpg" alt="模式2.2 语义取物"></a>
<br><sub><b>语义取物全流程</b> · 66 s · 2× 加速<br>观察 → 受限决策 → 对位 → 抓取 → 举示 → 放回 → 退离</sub>
</td>
<td width="40%" align="left">
<b>看这段视频时可以留意三点：</b><br><br>
① 机器人始终只做"选中的那一个目标"：四个候选同时存在，但只抓 `obj_1`；<br>
② 举示阶段有明确的 2 s 保持，随后是原位放回而不是随手丢下；<br>
③ 退离后 YOLO 重新检出四目标——证明物体确实离开了夹爪并稳定落在台面。
</td>
</tr>
</table>

### 复现

虚拟机侧（需先安装 `rosbridge_server` 并重新构建）：

```bash
cd /home/jiayuexie/ROS
source /opt/ros/melodic/setup.bash
catkin_make -j2 -l2
source /home/jiayuexie/ROS/devel/setup.bash
roslaunch final_project mode22.launch
```

Windows 侧先起本地模型服务，再起操作台：

```powershell
Start-Process -FilePath 'E:\Project\llama.cpp-b11836-win-cuda12\bin\llama-server.exe' -ArgumentList @(
  '-m','E:\Project\models\Qwen3-VL-4B-Instruct-GGUF\Qwen3VL-4B-Instruct-Q4_K_M.gguf',
  '--mmproj','E:\Project\models\Qwen3-VL-4B-Instruct-GGUF\mmproj-Qwen3VL-4B-Instruct-F16.gguf',
  '-c','4096','-np','1','--image-min-tokens','1024','-ngl','99','--host','127.0.0.1','--port','8080')

& 'E:\Programming\.conda\envs\mode22\python.exe' `
  'F:\University\Course\dasanshang\ROS_class\ROS\src\final_project\embodied\app.py' `
  --ros-host 192.168.162.132 --server-port 7860
```

> **注意**：`app.py` 和 README 里默认的 `--ros-host` 是 `192.168.162.131`，这次实测虚拟机是 `.132`，运行时用参数覆盖。把这个差别记下来，是免得把"连不上"错当成 ROS 节点出问题。

---

## 四、基础任务与其余入口

| 模块 | 做了什么 | 验收结果 | 证据 |
| :--- | :--- | :--- | :--- |
| **task1** Hello ROS | 最小节点：初始化 → `loginfo` → 退出 | 输出一次 `Hello ROS!`，进程正常结束 | `docs/media/tasks/task1_hello_ros.png` |
| **task2** 通信机制 | `Student` 消息（4 字段）、`QueryStudent` 服务、1 Hz 话题、参数服务器 | 1 Hz 稳定；`found=true/false` 两路正确；学号前导零在消息/YAML/命令行三处同时保住 | `docs/media/tasks/task2_*.png` |
| **task3** TF2 + PID | 双广播器 + 相对位姿 + 双通道 PID（微分低通、条件积分、朝向限速、滞回停车） | 20 Hz 控制，终距 0.094–0.098 m，到达后不抖 | `docs/media/tasks/task3_follow_*.jpg` |
| **模式1** 键盘 | R 收臂解锁、W/S/A/D、T 速度档、1/2 站点、P/O 抓放、空格取消 | 全按键闭环；P/O 成功；空格同时清速度与取消动作 | `docs/media/mode1/*.png` |
| **模式3** 语音 | 讯飞流式听写 → 文本归一化 → 闭集解析 → 同一速度门控 | 四类指令接受；真实麦克风完成"状态→准备→微调前进→停止" | 见 `docs/RUNBOOK.md` 第 5 节 |

<p align="center">
<img src="docs/media/tasks/task2_05_topic_rate_1hz.png" width="32%" alt="1Hz">
<img src="docs/media/mode1/14_pick_success.png" width="32%" alt="模式1抓取">
<img src="docs/media/tasks/task3_follow_02.png" width="32%" alt="双龟跟随">
</p>

<p align="center"><sub>左：`rostopic hz` 稳定 1 Hz；中：模式1 P 成功抬起箱体；右：双龟跟随的相对位姿与速度</sub></p>

<details>
<summary><b>为什么四种入口能共用一套底层？</b></summary>

底盘速度只有这一个出口：`velocity_gate` 是 `/om_with_tb3/cmd_vel` 唯一的发布者，按模式在 `MANUAL / NAV / DOCK` 三种来源里挑一个，模式对不上就发零速度；手动命令缓存 0.3 s，导航/对位命令缓存 1.0 s，超时就算过期；线速度限制在 [−0.08, 0.18]，角速度限制在 [−0.7, 0.7]。最后还有一道激光保护：运动方向 ±0.35 rad 内检测到障碍（前进 0.14 m / 后退 0.29 m）就直接停车，扫描没数据或超过 1.0 s 没更新也停车。

这样一来，"安全"不再是每个入口各做一套，而是所有入口共用一个出口。所以模式1 的 R 互锁、模式2.2 的心跳停车、模式3 的 STOP 优先级能同时存在，不会互相打架。

</details>

---

## 五、AUV 进阶实践（成果佐证）

> **说明：这里只放比赛的成果材料，不含 AUV 的任何源代码。** 工程还要用于后面的比赛，代码、工程目录、模型权重和完整原始录像都没传，`.gitignore` 里写了对应的排除规则。

课程里学的 ROS 通信、TF、PID、launch、状态机和录制方法，放到真的水下机器人上，不只是"能跑通"就行，还得"可信"：水下没有 GPS，池壁反光加上水浑，摄像头看不清，正式比赛要求拔掉调试线自己跑。2026 年，这台自己做的 AUV 参加了中国机器人大赛暨 RoboCup 机器人世界杯中国赛的水下机器人赛项（水中巡游项目），在 5 m × 10 m 的水池里做了正式下水尝试。

<p align="center">
<img src="AUV/screenshots/14_competition_pool.png" width="49%" alt="比赛水池">
<img src="AUV/screenshots/08_lidar_wall_detection.png" width="49%" alt="雷达墙检测">
</p>

<p align="center"><sub>左：比赛水池中的前视画面（水体蓝绿，底部为 AUV 前部结构）；右：真实水下雷达帧的多墙 RANSAC 拟合</sub></p>

### 软件分层

```mermaid
flowchart LR
    S["/scan · 外压 · 飞控姿态 · 相机检测"] --> E["证据层<br/>带时间戳的 observation"]
    E --> P["policy.py<br/>纯策略：obs → 四元组 x,y,z,r"]
    P --> M["final_mission.py<br/>服务 · 时间戳 · 超时 · 锁存"]
    M --> C["pymavlink_control_node<br/>唯一飞控写者"]
    M --> R["记录器<br/>bag · avi · events.jsonl · config.json"]
```

三条约束贯穿设计：**证据不创造目标**（缺失、歧义、过期即返回空，不用旧值补）、**唯一写者**（所有运动请求经 `MotionCommand` 进入同一飞控桥接）、**失败锁存**（压力/航向/飞控/记录/回执任一失效即 `ABORTED`，不自动重跑）。

### 主要结果

| 项目 | 条件 / 输入 | 结果 |
| :--- | :--- | :--- |
| 纯策略测试 | `test_final.py`，35 项 | **35/35 通过**，0.985 s |
| 状态机演示 | 确定性观测序列 | 28.7 s 到达 `DISARMING` |
| 赛场会话 981 | 1918 条状态，99.2 s，深度 −0.001–0.838 m | 完成下潜 → `FORWARD` → `BACK` → 最终停在 `LEFT` |
| 赛场会话 31be | 1082 条状态，57.4 s | 47.32 s 航向修正超时，锁存 `ABORTED` |
| 雷达墙检测 | 217c bag，850–862 s 窗口 | 稳定 1–3 面墙，最佳帧 0.459 / 1.054 / 2.445 m，RMS 7.3 / 7.9 / 13.0 mm |
| 球检测 | 217c 左目，141.5–147.0 s | 16 次 `blue_ball` 检测（置信度 0.38–0.84），**0 次通过门控** |

<p align="center">
<img src="AUV/screenshots/05_mission_demo.png" width="49%" alt="状态机演示">
<img src="AUV/figures/mission_timeline_981ac8a2.png" width="49%" alt="赛场时间线">
</p>

<p align="center"><sub>左：状态机确定性演示；右：真实赛场会话 981ac8a2 的状态、深度、航向与请求时间线</sub></p>

### 三个值得记下来的问题

1. **有闭环不等于能收敛。** 31be 会话中目标航向约 70°，偏航速率请求从 −158 一路衰减到 −107，30 s 内误差始终没回到 −1°，适配层捕获超时后锁存中止。推进器推力、初始角速度、飞控限幅与水流都会让闭环失败。
2. **归一化宽高比是一个坐标系统陷阱。** 16:9 画面中归一化宽高比等于像素宽高比乘以 0.5625；真实蓝球的归一化宽高比为 0.306–0.425，全部被 0.5–2.0 的门控拒绝，`ball_x` 始终为空。问题不在"模型没检出"，而在"检测框进入策略的几何条件写错了坐标系"。
3. **"未确认"必须写进日志。** 前进结束记 `impact unconfirmed`、后退结束记 `rear contact unconfirmed`、左移结束记 `return unconfirmed`——定时到、近墙或球消失都不等于撞球成功。

### 材料索引

| 目录 | 内容 | 数量 |
| :--- | :--- | :--- |
| `AUV/figures/` | 架构图、任务时间线、航向中止、雷达三面墙拟合、球门控诊断 | 5 张 |
| `AUV/screenshots/` | 环境、工程结构、单元测试、状态机、墙检测、球检测、赛场 | 13 张 |
| `AUV/videos/` | 入水与下潜、状态机终端演示、雷达墙检测动画、蓝球检测标注、比赛水下片段 | 5 段 |
| `AUV/logs/` | 单元测试、状态机、真实会话摘要、墙/球检测、环境核验 | 25 份 |
| `AUV/data/` | 会话 CSV、雷达 JSONL、检测 CSV、演示参数副本 | 7 份 |
| `AUV/AUV_Project_Summary.md` | 完整文字版：算法、实验、图表索引、复现路径 | 1 篇 |

<p align="center">
<a href="AUV/videos/05_pool_launch_and_dive.mp4">▶ 入水与下潜（35 s）</a> ·
<a href="AUV/videos/04_competition_pool.mp4">▶ 比赛水下片段</a> ·
<a href="AUV/videos/02_lidar_wall_detection.mp4">▶ 雷达墙检测</a> ·
<a href="AUV/videos/03_ball_detection_annotated.mp4">▶ 蓝球检测标注</a> ·
<a href="AUV/videos/01_mission_state_machine.mp4">▶ 状态机演示</a>
</p>

---

## 六、复现与验证

**环境准备**（每个新终端）：

```bash
export LANG=C.UTF-8
export LC_ALL=C.UTF-8
export PYTHONIOENCODING=utf-8
source /opt/ros/melodic/setup.bash
source ~/ROS/devel/setup.bash
```

**构建**：

```bash
cd ~/ROS
catkin_make -j2 -l2
```

**四个入口**：

| 入口 | 启动命令 |
| :--- | :--- |
| 模式 1 键盘 | `roslaunch final_project simulation.launch mode:=1` 然后 `rosrun final_project keyboard_control.py` |
| 模式 2.1 全自动 | `roslaunch final_project simulation.launch mode:=2 target_color:=red` |
| 模式 2.2 语义 | `roslaunch final_project mode22.launch` + Windows 操作台 |
| 模式 3 语音 | `roslaunch final_project simulation.launch mode:=3` |

逐条命令、按键语义、话题与服务清单见 [`docs/RUNBOOK.md`](docs/RUNBOOK.md)；逐轮部署、失败与修复记录见 [`ROS/src/final_project/VALIDATION.md`](ROS/src/final_project/VALIDATION.md)。

> 本仓库所有数值都来自**单次完整运行**的日志与 `run.json`，不用于推断成功率。抓取与放置验收使用 Gazebo 模型状态，适合仿真，不等于真实传感器精度。

---

## 七、仓库说明

### 收录范围

| 已收录 | 说明 |
| :--- | :--- |
| `ROS/src/**` | task1、task2、task3、final_project 全部源码、launch、config、URDF、地图与 ONNX 权重 |
| `docs/media/**` | 模式 2.1 / 2.2 / 模式1 / task1–3 的截图与经加速裁剪的演示视频 |
| `AUV/**` | 图表、截图、日志、数据与 5 段演示视频（**仅佐证材料**） |

| 未收录 | 原因 |
| :--- | :--- |
| AUV 源代码（`Jetson_core_code/`、`tools/`、`ros_home/`） | 后续比赛在研项目，禁止公开 |
| AUV 原始录像与完整媒体库（约 878 MB） | 体积过大，且含未脱敏现场信息 |
| 课程报告 docx | 含姓名、学号等个人信息 |
| catkin `build/`、`devel/`、`logs/`、`backups/` | 构建产物与历史快照，不含源信息 |

### 第三方复用与许可

自己写的部分有手动 Action、四种模式的接入、控制权切换、运动学校验、带条件判断的抓取插件，还有模式 2 里对探索做的改动和"建图—定位—取送"的先后协调。用到的第三方东西保留原来的许可：

- `urdf/upstream/*.xacro`、`config/robot.srdf`、ALVAR 标记贴图：ROBOTIS，Apache-2.0
- `src/third_party/explore_lite/`：`hrnr/m-explore`，BSD 三条款（含 Bosch / Jiri Horner 版权）
- `vision/`：Ultralytics YOLOE 导出链路，AGPL-3.0（许可证随目录提供）
- GMapping、AMCL、move_base、MoveIt、ros_control、Gazebo：ROS Melodic 发行包
- 模式 2.2：Qwen3-VL-4B-Instruct-GGUF、llama.cpp、rosbridge_suite、roslibpy、Gradio

完整清单见 [`ROS/src/final_project/THIRD_PARTY.md`](ROS/src/final_project/THIRD_PARTY.md)。

### 已知限制

- 模式 2.1 的结果来自一次完整运行，还是对定位比较敏感；B 台放置误差 0.07858 m 已经接近 0.08 m 的容差，说明松手时的接触和台面滑动还没算进去。
- 模式 2.2 只跑过一次语义取物，反向任务、连着做多次、动态障碍和真实硬件都还没试；Qwen 推理要 2.982 s，心跳是 3 s，这套演示的快慢基本就由这两个数定下来。
- 模式 3 的完整语音搬运与运动中语音停车尚未验收；云端识别存在延迟，精对位仍依赖微调指令。
- AUV 部分为"真实链路已跑通、完整任务未完成"的如实记录，不宣称赛项满分。

---

<div align="center">
<sub>本项目为《Linux 基础及 ROS 应用实践》课程作业 · 仅收录课程成果与脱敏后的比赛佐证材料</sub>
<br>
<sub>如果这个仓库对你有帮助，欢迎 star ⭐</sub>
</div>
