# M6 diagram and image sources / 图片素材说明

本目录新增图片为课程原创原理示意，使用同一份几何定义生成 `.drawio`、`.svg` 和 `.png`。图示不是实验测量结果。当前环境没有 draw.io 桌面 CLI，PNG 由 Pillow 按相同节点与连线渲染；`.drawio` 保留可编辑节点，SVG 保留矢量内容。公式曲线由 Matplotlib 生成。

These are authored teaching illustrations, not measured results. Draw.io, SVG, and PNG share a geometry definition. With no desktop draw.io CLI available locally, Pillow renders PNG; draw.io sources retain editable nodes. Matplotlib generates formula illustrations.

| 文件 / File | 用途 / Purpose |
| --- | --- |
| `navigation_architecture.*` | M3/M5 到 M6 的系统架构 / system architecture |
| `mapping_workflow.*` | 建图、保存、重定位、导航阶段 / phase flow |
| `planning_loop.*` | 全局路线与局部控制 / planning and control loop |
| `lidar_detection.*` | 点云检测流程 / LiDAR detection |
| `multi_robot.*` | 待集成的多机通信架构 / proposed multi-robot architecture |
| `planning_principles.png` | 膨胀代价与候选轨迹示意 / formula illustration |
| `formation_geometry.png` | 编队目标几何 / formation geometry |

从仓库根目录重新生成 / regenerate from repository root:

```bash
/usr/bin/python3 docs/M06-Localization-Navigation-and-Path-Planning/images/generate_diagrams.py
```

脚本依赖 Pillow、NumPy、Matplotlib 和 DejaVu Sans 字体；这些是文档制图依赖，不是新增导航运行节点。图的箭头为教学层面的数据 / 控制关系，具体 ROS Action、Topic、TF 以各节接口表和源码为准。

课程效果图继续引用 `modules/m06_navigation/j501_navigation/assets/` 中已有的 Gazebo、RViz、规划、检测与建图对比图，其数据出处在对应章节和 `validation/` 中标明。未从互联网复制第三方图片；参考资料链接用于阅读算法与版本说明。
