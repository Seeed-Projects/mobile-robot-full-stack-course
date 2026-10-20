# M07 底盘 VLN 最小复现材料

[English](README.md) | [中文](README_zh_CN.md)

[English](README.md) | [中文](README_zh_CN.md)

本目录把**固定LightNav上游＋课程覆盖层＋外部二进制与CAD**分开组织。内容用于Jetson上的DM-H65纯仿真；源码冻结自2026-10-15现场，不会自动安装依赖、下载模型或运行导航。

## 冰箱课程与自定义底盘

新版M7按VLN理论、LightNav算法与Jetson Demo运行组织，CAD接入与首次安装见完整实验手册。冰箱视频是17次预测、模型STOP的未评分自由任务展示，不与下方5次预测旧门框记录混算。运行入口见[7.3](../7.3_From_Navigation_Prediction_to_Closed_Loop_Control/README_zh_CN.md)。

新增[自定义底盘教学材料](examples/custom-chassis/README_zh_CN.md)提供只读URDF清单、部件映射和独立MJCF预览。它不会把任意CAD自动接入冻结DM-H65加载器，也不会改写overlay或在线项目。

## 阅读顺序

从[M7实验手册](../7.3_From_Navigation_Prediction_to_Closed_Loop_Control/LAB_GUIDE_zh_CN.md)开始。不要直接把本目录复制进在线项目，也不要运行overlay里的其他历史部署入口。

| 内容 | 作用 |
| --- | --- |
| `overlay/` | 相对上游项目的源码覆盖层，包含模型部署适配、底盘后端、历史、几何跟踪、网页和相关测试 |
| [overlay-manifest.json](overlay-manifest.json) | 每份文件的SHA256和上游原文件SHA256，锚定commit |
| `locks/` | 模型与仿真环境分别锁定；保证不同，见下文 |
| `configs/` | 原始底盘、几何profile、任务登记与CAD清单；含历史绝对路径，准备工具在副本中迁移 |
| [evidence/](evidence/README_zh_CN.md) | 底盘真实原图、响应、位姿和历史GPU记录 |
| `scripts/m7/` | 材料检查、项目准备和Demo启动／停止入口 |
| `examples/` | 自定义底盘部件检查与独立预览 |
| `teaching/` | 教学动画制作与离线复算 |
| `tests/` | 课程工具回归测试 |
| `scripts/m7/` | 材料检查、项目准备和Demo启动／停止入口 |
| `examples/` | 自定义底盘部件检查与独立预览 |
| `teaching/` | 教学动画制作与离线复算 |
| `tests/` | 课程工具回归测试 |
| `tools/` | 只读材料检查、离线分析及显式应用的源码/资产准备工具 |
| [第三方说明](THIRD_PARTY_zh_CN.md) | 来源、独立资产、历史记录边界 |

固定上游commit：`3015508b70fb6e30bbd66701e94c9b916dcb8df8`。固定模型revision：`826dc5fbfa37afa8293d2e336d329b6ffc0bfb64`。覆盖层不是完整LightNav仓库，也不是另一个VLM。

源码依赖上游未修改模块，包括模型配置、RVQ、场景加载、robot base、CasADi模块等。虽然Demo不实例化MPC，一些共享类型模块仍导入其依赖，所以不能自行删掉锁里的包。底盘配置读取的是几何JSON，不要求安装或运行真实SDK。

## 完整性与准备命令

本目录统一位于M7的 `code/` 下。以下命令在课程仓库根目录运行：

```bash
python3 docs/M07-Vision-Language-Navigation/code/tools/check_materials.py
python3 -m unittest discover -s docs/M07-Vision-Language-Navigation/code/tests -v
```

安装覆盖层和准备资产的命令默认dry-run，只有明确加`--apply`才写新项目。现有改动不符合上游原始哈希时拒绝，避免覆盖个人工作。

```bash
python3 docs/M07-Vision-Language-Navigation/code/tools/install_overlay.py --project /path/to/fresh/LightNav-0
python3 docs/M07-Vision-Language-Navigation/code/tools/prepare_assets.py \
  --project /path/to/fresh/LightNav-0 --bundle /path/to/dm-h65-course-assets-20261015.tar.gz
```

包装脚本从自身位置定位课程仓库，默认准备项目仍在仓库根目录的 `shared/lightnav/LightNav-0`；已有设备项目请显式使用 `--project`。

包装脚本从自身位置定位课程仓库，默认准备项目仍在仓库根目录的 `shared/lightnav/LightNav-0`；已有设备项目请显式使用 `--project`。

这是课程材料接口，不修改在线系统的公共API。在线Demo继续使用8050模型服务、8091网页、`/api/health`与`/ws`。网页启动消息为`start_sim_navigation`，不是旧严格模式的`start_step_navigation`；模型连接继续使用login/reset/next。

## 环境锁与外部二进制

模型锁[requirements-lightnav-py310.lock](locks/requirements-lightnav-py310.lock)包含传递依赖及包哈希；两个file URL指向原部署路径的本地wheel。仿真锁[requirements-demo-cpu.lock](locks/requirements-demo-cpu.lock)是原运行环境的精确版本锁，**不含每个wheel哈希**且有`-e .`，必须在新项目的`mujoco_demo`目录运行`uv pip sync`，详见手册，不运行整个项目的`uv sync`。

| 外部文件 | 固定SHA256 |
| --- | --- |
| torch-2.7.0-cp310-cp310-linux_aarch64.whl | `6eff643c0a7acda92734cc798338f733ff35c7df1a4434576f5ff7c66fc97319` |
| torchvision-0.22.0-cp310-cp310-linux_aarch64.whl | `daabff3a0725996886b92e4b5dd143f5750ef4b181b5c7d01371a9185e8f0402` |
| libcusparse_lt-linux-aarch64-0.8.1.1_cuda12-archive.tar.xz | `5426a897c73a9b98a83c4e132d15abc63dc4a00f7e38266e7b82c42cd58a01e1` |

前两项是项目测得的下载物哈希，cuSPARSELt核对NVIDIA元数据。wheel来源为[Seeed教程](https://github.com/Seeed-Projects/reComputer-Jetson-for-Beginners/tree/main/3-Basic-Tools-and-Getting-Started/3.5-Pytorch)，模型从[固定HF revision](https://huggingface.co/LightOriginsHQ/LightNav-0/tree/826dc5fbfa37afa8293d2e336d329b6ffc0bfb64)取得。不把归档成功记录当成新设备GPU校验结果。

## 外部CAD与视频

资产包名、大小、SHA256见[asset-bundle.json](configs/asset-bundle.json)。本轮本机另存教师资产包，包含45份网格，不纳入Git，也未建立公共下载地址。学生缺少该包时，新机DM-H65入口仍被资产准备阻塞；可先进行离线证据练习。

原速视频作为独立教师材料提供，由教师提供的材料清单定位。视频来自历史失败任务，不标成成功案例，也不在课程Git中加入模型或大日志。

## 版本与验证边界

- 覆盖层哈希来自当前Jetson；历史证据来自更早的runtime-4版本，两者不得当成同一构建。
- 当前代码支持有限局部修复和旧短路径重新观察；历史runtime-4未启用修复。
- 本轮验证材料校验、准备工具、防覆盖、配置迁移、离线日志及文档，没有在新板重装环境或执行导航。
- 不修改系统组件，不启动实机SDK或设备连接，不commit、不push。

## SlowFast可视化教学材料

7.2.3配套[两段GIF与三张静态图的离线制作/复算入口](teaching/slowfast/README.md)。使用固定选帧源码与教学特征网格；不改变冻结运行覆盖层，不要求加载模型或连接Jetson。

## LightNav-0算法动画教学材料

7.2.2、7.2.4～7.2.7配套[五段GIF与五张静态分解图的离线制作/复算入口](teaching/lightnav/README.md)。指向编码、SE(2)复合与RVQ解码使用固定上游函数和发布bundle的冻结算例；不改变冻结运行覆盖层，不要求加载模型、连接Jetson或生成新插画。

