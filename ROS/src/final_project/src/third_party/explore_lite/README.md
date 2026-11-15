# explore_lite边界搜索复用

来源：`hrnr/m-explore` 的 `lunar-devel` 分支，读取日期2026-10-20。

- https://github.com/hrnr/m-explore/tree/lunar-devel/explore
- https://raw.githubusercontent.com/hrnr/m-explore/lunar-devel/explore/src/frontier_search.cpp
- https://raw.githubusercontent.com/hrnr/m-explore/lunar-devel/explore/include/explore/frontier_search.h
- https://raw.githubusercontent.com/hrnr/m-explore/lunar-devel/explore/include/explore/costmap_tools.h

仅保存边界搜索、邻域/最近单元工具及相关声明，不创建第二个catkin包，不复制map_merge。BSD许可与原版权声明保存在同目录 `LICENSE.BSD`，来源为同包 `src/explore.cpp` 的许可头。

本地改动：补充直接使用的标准库/消息头；将初始边界单元纳入points、质心与最近距离计算，修正原size包含首单元但求和遗漏的偏差。连通搜索、聚类及排序逻辑保留。上游explore.cpp的调度没有复制；本包 `src/automatic_explore.cpp` 负责空闲区目标、安全间距、规划检查、有限重试、完成/受阻状态和可视化。

源码随本项目保存，后续构建不依赖联网下载或额外安装explore_lite。算法复用不作为原创算法宣称。
