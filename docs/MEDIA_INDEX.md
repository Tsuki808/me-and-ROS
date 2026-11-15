# 媒体索引与溯源

README 中的图片与视频统一使用 ASCII 文件名，便于 URL 稳定与跨平台访问。下表保留**原始中文文件名**与内容说明，便于回溯到本地证据目录。

原始目录：`图片/final_project_模式2.1/`、`图片/final_project_模式2.2/`、`图片/final_project_模式1/`、`图片/task1|task2|task3/`。

---

## 一、模式 2.1（`docs/media/mode2_1/`）

| 现文件名 | 原始文件名 | 内容 |
| :--- | :--- | :--- |
| `01_startup_rviz.png` | 01_启动_RViz建图开始.png | RViz 同时显示地图、激光、RGB-D、深度与点云 |
| `02_yolo_window.png` | 02_启动_YOLO观察窗口.png | YOLOE-26n 观察窗口与推理耗时 |
| `03_mapping_mid.png` | 03_建图中期_RViz地图.png | 建图中期：墙体轮廓与激光束 |
| `04_frontier_status.png` | 04_建图中期_frontier状态.png | 探索状态终端：frontier 递减、failures=0 |
| `05_mapping_late.png` | 05_建图后期_RViz地图.png | 建图后期：地图主体完成 |
| `06_map_save_amcl.png` | 06_地图保存_AMCL交接_状态.png | MAPPING→SAVING_MAP→LOCALIZING→DELIVERING |
| `07_yolo_four_boxes.png` | 07_A台YOLO多目标检测.png | A 台同帧检出红/蓝/黄/绿四盒 |
| `08_dock_before_pick.png` | 08_A台抓取前_夹爪对准红盒.png | 夹爪张开对准红盒 |
| `09_pick_success.png` | 09_A台抓取成功_携带红盒.png | 红盒被夹持，顶部 ALVAR 清晰 |
| `10_carry_to_b_rviz.png` | 10_携带导航至B_RViz.png | 携带红盒导航至 B 台 |
| `11_b_place.png` | 11_B台放置_盒子落桌.png | 红盒落桌、夹爪张开 |
| `12_b_retreat.png` | 12_B台放置完成_退离.png | 盒子留在 B 台，夹爪退离 |
| `13_return_home.png` | 13_返航停车_RViz地图.png | 返航待命区，最终地图 |
| `14_final_acceptance.png` | 14_终态验收_终端.png | SUCCEEDED / HOLD / 位姿 / 误差全量 |
| `15_final_gazebo.png` | 15_终态_Gazebo迷宫与机器人.png | 迷宫与机器人终态 |
| `16_status_timeline.png` | 16_状态话题_SUCCEEDED.png | `/final_project/status` 完整阶段时序 |
| `17_moveit_log.png` | 17_launch日志_MoveIt执行.png | MoveIt 规划与轨迹执行日志 |
| `18_mapping_sheet.jpg` | 18_建图序列联络表.png | 13 帧建图联络表 |
| `19_pick_sheet.jpg` | 19_A台动作联络表.png | 15 帧 A 台动作联络表 |
| `20_place_sheet.jpg` | 20_B台动作联络表.png | 15 帧 B 台动作联络表 |
| `21_grasp_envelope_fail.png` | 21_失败分析_35mm包络.png | 第一轮 35 mm 包络失败现场 |
| `22_model_correction.png` | 22_修复日志_模型校正.png | 模型一致性校正日志 |
| `23_final_map.png` | 23_最终地图.png | 352×672、0.025 m 栅格地图 |
| `24_workspace_overview.png` | 24_工作台总览.png | Gazebo + RViz + YOLO + 终端同屏 |
| `25_selected_frame.jpg` | 25_A台选盒原始帧.jpg | 未叠加检测框的原始选盒帧 |
| `26_fullscreen_sheet.jpg` | 26_用户补充全屏现场_8帧.png | 8 帧 VMware 全屏现场联络表 |

## 二、模式 2.2（`docs/media/mode2_2/`）

| 现文件名 | 原始文件名 | 内容 |
| :--- | :--- | :--- |
| `01_startup_rviz.png` | 01_启动_RViz综合显示.png | 启动后的 RViz 综合显示 |
| `02_yolo_window.png` | 02_启动_YOLO观察窗口.png | YOLO 观察窗口 |
| `03_four_candidates.png` | 03_观察位_YOLO四候选检测.png | 同帧四候选检测 |
| `04_observation_rate.png` | 04_启动_ROS节点与观察频率.png | 节点列表与 2.6–2.8 Hz 观察频率 |
| `05_console_initial.png` | 05_操作台_初始界面.png | Windows 操作台初始界面 |
| `06_console_guard.png` | 06_操作台_未分析执行拦截.png | 未分析直接执行被拦截 |
| `07_qwen_decision.png` | 07_Qwen决策_黄盒obj_1.png | Qwen 决策与推理耗时 |
| `08_observation_json.png` | 08_观察帧与结构化决策.png | 观察帧与结构化 JSON |
| `09_pick_stage.png` | 09_执行中_PICK阶段.png | PICK 阶段控制台反馈 |
| `10_dock_before_pick.png` | 10_抓取前_夹爪对准黄盒.png | 夹爪对准黄盒 |
| `11_pick_success.png` | 11_抓取成功_黄盒离台.png | 黄盒离台 |
| `12_show_hold.png` | 12_举示_黄盒保持2秒.png | 举示保持 2 s |
| `13_place_back.png` | 13_放回_黄盒归位A台.png | 原位放回 |
| `14_detection_resume.png` | 14_放回后_YOLO恢复检测.png | YOLO 恢复检测 |
| `15_four_boxes_after.png` | 15_退离后_四目标恢复检测.png | 退离后四目标恢复 |
| `16_rviz_show.png` | 16_RViz_举示阶段综合显示.png | RViz 举示阶段 |
| `17_status_timeline.png` | 17_状态话题_阶段时序.png | 阶段时序 |
| `18_moveit_log.png` | 18_launch日志_MoveIt规划执行.png | MoveIt 规划执行日志 |
| `19_final_acceptance.png` | 19_终态验收_话题与服务.png | 终态验收 |
| `20_final_ui.png` | 20_最终UI_任务完成.png | 最终操作台 |
| `21_action_sheet_yolo.jpg` | 21_动作序列联络表_YOLO相机.png | YOLO 相机动作联络表 |
| `22_final_rviz.png` | 22_仿真终态_RViz.png | 仿真终态 RViz |
| `23_final_yolo.png` | 23_仿真终态_YOLO.png | 仿真终态 YOLO |
| `24_action_sheet_rviz.jpg` | 24_动作序列联络表_RViz.png | RViz 动作联络表 |

## 三、模式 1 与基础任务（`docs/media/mode1/`、`docs/media/tasks/`）

| 现文件名 | 原始文件名 |
| :--- | :--- |
| `mode1/01_startup_gazebo.png` | 01_模式1启动_Gazebo世界.png |
| `mode1/02_startup_rviz.png` | 02_模式1启动_RViz地图.png |
| `mode1/04_arm_extended.png` | 04_R前_机械臂展开.png |
| `mode1/05_arm_stowed.png` | 05_R后_机械臂收臂.png |
| `mode1/07_drive_ready.png` | 07_R后键盘提示_可驾驶.png |
| `mode1/09_fine_mode.png` | 09_T_FINE_速度档.png |
| `mode1/12_align_prompt.png` | 12_A台对位_键盘提示.png |
| `mode1/13_grasp.png` | 13_P抓取_夹爪套盒.png |
| `mode1/14_pick_success.png` | 14_P成功_箱体抬起.png |
| `mode1/18_place_success.png` | 18_O成功_箱体在B台.png |
| `mode1/20_cancel.png` | 20_空格取消_R_MANUAL_CANCELLED.png |
| `mode1/21_space_stop.png` | 21_空格停车_驾驶中.png |
| `tasks/task1_hello_ros.png` | task1/051fc68901932abd7d9b85a8d26357f3.png |
| `tasks/task2_01_build_msgs_srv.png` | 01_构建_自定义消息与服务生成.png |
| `tasks/task2_02_launch_three_nodes.png` | 02_communication_launch_三节点启动.png |
| `tasks/task2_03_rosmsg_rossrv.png` | 03_自定义消息与服务接口_rosmsg_rossrv.png |
| `tasks/task2_04_topic_one_frame.png` | 04_话题单帧_四字段完整.png |
| `tasks/task2_05_topic_rate_1hz.png` | 05_话题频率_稳定1Hz.png |
| `tasks/task2_06_pub_sub_graph.png` | 06_话题发布订阅关系.png |
| `tasks/task2_07_service_list.png` | 07_服务列表与接口信息.png |
| `tasks/task2_08_param_server.png` | 08_参数服务器_学生资料.png |
| `tasks/task2_09_query_known_found_true.png` | 09_客户端查询已知学号_found_true.png |
| `tasks/task2_10_query_unknown_found_false.png` | 10_客户端查询未知学号_found_false.png |
| `tasks/task2_11_rosservice_known.png` | 11_服务调用已知学号.png |
| `tasks/task2_12_rosservice_unknown.png` | 12_服务调用未知学号_默认消息.png |
| `tasks/task2_13_publisher_info.png` | 13_发布节点通信信息.png |
| `tasks/task2_14_subscriber_info.png` | 14_订阅节点通信信息.png |
| `tasks/task2_15_service_info.png` | 15_服务节点通信信息.png |
| `tasks/task2_16_package_cmake.png` | 16_包结构与CMake接口配置.png |
| `tasks/task2_17_msg_srv_files.png` | 17_消息与服务源文件.png |
| `tasks/task2_18_rqt_graph.png` | 18_rqt_graph_节点话题关系.png |
| `tasks/task2_19_quote_error_fix.png` | 19_服务调用引号错误与修正.png |
| `tasks/task3_follow_01.jpg` | task3/1997bc30d71014f1c737394a5b4cdd58.jpg |
| `tasks/task3_follow_02.png` | task3/3863bda2f2d665750f915219fb7ef4ac.png |
| `tasks/task3_follow_03.jpg` | task3/8e013c83ee2edad6df6303c58f04b1a5.jpg |

## 四、演示视频（`docs/media/video/`）

原始素材为 2 fps 屏幕录像（模式 2.2 为 6 fps）。上传前统一处理为 **2× 加速 + 核心片段截取 + H.264 CRF 28**，并去除音轨。

| 现文件名 | 原始文件 | 处理方式 | 输出 |
| :--- | :--- | :--- | :--- |
| `mode2_1_pick.mp4` | 模式2.1_A台抓取.mp4（173 s） | 取 55–173 s，2× | 59 s / 0.8 MB |
| `mode2_1_place.mp4` | 模式2.1_B台放置.mp4（323 s） | 取 200–323 s，2× | 61.5 s / 0.9 MB |
| `mode2_1_mapping.mp4` | 模式2.1_建图延时.mp4（14 s） | 全片 2× | 7 s / 0.3 MB |
| `mode2_2_semantic_pick.mp4` | 模式2.2_动作录像.mp4（131 s） | 全片 2× | 65.7 s / 0.8 MB |

每段视频同名 `*_poster.jpg` 为封面帧，README 中作为可点击缩略图使用。

> 完整原始录像（`原始证据/` 下的`.avi`，单文件最大 203 MB）未入库：GitHub 单文件上限 100 MB，且与已上传的加速版内容重复。
