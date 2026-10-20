# 来源与材料边界

[English](THIRD_PARTY.md) | [中文](THIRD_PARTY_zh_CN.md)

- `overlay/`以LightNav-0固定提交为基线，包含本项目的部署与底盘仿真修改。上游许可证副本为[LightNav-LICENSE](licenses/LightNav-LICENSE)，原第三方声明为[LightNav-THIRD_PARTY_NOTICES.md](licenses/LightNav-THIRD_PARTY_NOTICES.md)。原声明保留全貌，不代表课程包含其中全部组件。
- ProcTHOR/MolmoSpaces场景由上游提供，遵循其独立资产条款与署名要求；不把代码许可证直接套到场景。课程不复制整个数据集。
- CAD来自用户提供的Onshape装配导出，保留原始导出SHA256、简化网格清单和参数来源。另存教师包供本项目复现，不由此声称已经获得公共再分发许可。
- NVIDIA/Seeed wheel、cuSPARSELt、HF权重由原发布渠道提供，课程仅提供版本、校验和准备步骤。
- 新课程工具是离线准备与分析工具，不依赖或访问真实SDK。`sdk-chassis.json`只是冻结几何与方向配置，不含设备会话、口令或访问令牌。
- 本课程没有改变仓库根目录尚未最终选定的许可方案，不擅自将全部教材、CAD和第三方代码统一声明为同一许可证。
