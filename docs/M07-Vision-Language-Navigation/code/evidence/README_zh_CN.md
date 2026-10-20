# 真实历史证据：DM-H65 底盘导航

[English](README.md) | [中文](README_zh_CN.md)

这些是既有记录的副本，不是本轮课程改写新运行的结果。

## 主样例

原始运行：`sim-demo-20261013/evidence/remote/runtime-4/navigation-0001`，2026-10-13。五次预测、四次安装；第五条危险路径被拒绝，`strict_success=false`。该版本不含后续局部修复。

- [registration.json](registration.json)：原指令、起点、限制和评分上下文。
- [outcome.json](outcome.json)：完整原始结果及统计。
- [historical-summary.json](historical-summary.json)：已有摘要。
- [installed-paths.json](installed-paths.json)：同次任务四条实际安装参考的原始事件。
- [actual-state.jsonl](actual-state.jsonl)：按`task_id=navigation-0001`从父运行日志提取的全部状态行；无插值、无改写。分析工具进一步限定active窗口，排除任务结束后仍带同一ID的空闲摘要。
- [prediction-01/raw.json](prediction-01/raw.json)与[input.jpg](prediction-01/input.jpg)：第一次预测全部响应、时间、外参和哈希。
- prediction-02至05同样保留真实原图、响应及路径检查；没有把局部修复值补入不存在的历史字段。

从课程根目录重算：

```bash
python3 docs/M07-Vision-Language-Navigation/code/tools/analyze_run.py docs/M07-Vision-Language-Navigation/code/evidence \
  --state docs/M07-Vision-Language-Navigation/code/evidence/actual-state.jsonl --task-id navigation-0001
```

报告中的success仍是既有outcome记录值，工具不重新证明两秒严格评分。即使状态日志没有接触记录，也不能仅凭稀疏记录声称完整工业安全认证。

## GPU与模型记录

[gpu-gate1.json](gpu-gate1.json)、[gpu-operations.json](gpu-operations.json)和[model-manifest.json](model-manifest.json)为历史部署记录，含当时设备路径；对应大trace和权重未收录。记录说明历史版本已做过验证，不能据副本给新设备签发PASS。

## 后续实现

2026-10-14新增有限局部修复，2026-10-15修复旧短路径追赶、遥测和评分显示。现有后续试验仍没有证明稳定严格成功；正文按版本分别介绍，不将后续修复代码解释成主样例已经运行过的功能。
