# 底盘导航复核摘要（2026-10-15）

本摘要来自既有导航复核报告，未产生新推理或新运行。原报告本机路径为 `custom-chassis-deployment/sim-demo-20261013/NAVIGATION_AUDIT_20261015.md`。

- 修复过捕获时旧短路径追赶、旧OPOS显示与控制续约/遥测耦合。
- 冰箱复测指令为 `Find the refrigerator in the room and stop beside it.`，8次预测、约29.43s，1次局部修复，最后危险路径拒绝，未实际碰撞。
- 门框复测另有11次预测、46.34s、模型STOP但距门框评分点约2.28m的失败；不能用该距离评价冰箱。
- 对象清单未包含refrigerator，不代表场景不存在冰箱；原图确实可见灰色冰箱。
- 本摘要对应复核记录，不对应课程car.mp4中的17次预测，不能补作视频缺少的原始日志。

原报告SHA256：`1788205e4da8c39dfd7ada788827b12cbcf78033379f77399b4d87be9f8308c7`。
