# 自定义底盘：从CAD检查到MuJoCo预览

[English](README.md) | [中文](README_zh_CN.md)

这里是课程侧教学材料，不会安装到在线项目，也不会改变已有控制器。先读[7.3](../../../7.3_From_Navigation_Prediction_to_Closed_Loop_Control/README_zh_CN.md)。

## 1. 明确三个完成程度

| 阶段 | 产出 | 尚未证明 |
| --- | --- | --- |
| CAD检查 | 部件、关节树、引用、单位记录、SHA256 | 场景碰撞和动力学 |
| MuJoCo预览 | 正确外观、尺寸、坐标、相机 | 模型驱动导航 |
| 房间闭环 | 新后端、真实输入、模型路径、执行与新观察 | 稳定成功率与实车一致性 |

冻结DM-H65适配器要求43个视觉link和两个驱动轮，不能把另一份URDF塞进其config后声称通用适配。

## 2. 只读检查真实URDF

```bash
python3 docs/M07-Vision-Language-Navigation/code/examples/custom-chassis/inspect_urdf.py \
  /path/to/export/robot.urdf \
  --package your_robot_description=/path/to/your_robot_description
```

相对路径按URDF所在目录解析；package URI必须显式映射，不联网下载。工具检查单根关节树、重复名、环、缺失mesh和逐文件哈希，输出到标准输出，不写源资产。

`valid_inventory=true`只表示清单引用检查通过，不是尺寸、自由度、碰撞、动力学或导航资格。STL的数值单位需要人工对照轮径和车体尺寸；工具不会猜测米和毫米。

## 3. 完成部件映射

复制[part-mapping.example.json](part-mapping.example.json)到自己的工作目录，填写真实部件名、轮轴中心、脚轮、几何尺寸和相机。它是设计记录模板，不是冻结加载器可直接消费的配置。

保留CAD原包；明确是否已把mesh变换到根坐标。对视觉和碰撞分别记录来源。驱动轮轴、脚轮转向轴与滚动轴不能由不透明文件名自动猜定。

## 4. 独立查看MJCF

[preview.xml](preview.xml)只含静态教学primitive，不含真实DM-H65网格，也没有驱动或控制。两大轮与四小轮说明部件关系，不能用它替代真实CAD后写“导入完成”。

在已安装MuJoCo的电脑上：

```bash
python -m mujoco.viewer --mjcf=/absolute/path/to/preview.xml
```

Jetson无桌面时用既有仿真环境独立渲染，不启动模型或在线Demo：

```bash
MUJOCO_GL=egl /path/to/mujoco-env/bin/python - <<'PY'
import mujoco
from PIL import Image
model = mujoco.MjModel.from_xml_path('/absolute/path/to/preview.xml')
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)
with mujoco.Renderer(model, height=270, width=480) as renderer:
    renderer.update_scene(data, camera='ego')
    Image.fromarray(renderer.render()).save('/tmp/my-chassis-ego.png')
PY
```

相机前方只有空地是模板的预期结果。加入自己的授权mesh时，在asset中声明，再增加`type="mesh"`视觉geom；关闭视觉geom的碰撞，用独立几何覆盖实际外形。之后再检查场景模型、相机是否能看到任务目标。

## 5. 接入当前工程的接口清单

在新项目副本实现适配，不删冻结DM-H65校验。参考课程覆盖层`robots/dm_h65.py`与`sim_demo_runtime.py`：

- 模型构建：场景与底盘合并，根body/joint、碰撞geom、相机名可稳定查询。
- 状态：从同一个MuJoCo实例读取根位姿、实际速度、相机外参与接触。
- 运动学：X前/Y左、偏航CCW；更新状态后`mj_forward`，相机必须从新状态渲染。
- 几何：完整车身及转向部件包络，连续路径预检和运行时检查。
- 生命周期：STOP、RESET、取消代次、旧结果失效、路径TTL、断连。
- 显示与日志：模型原始输入输出、执行参考、实际状态分开记录。

这一步需要针对新底盘写后端，当前课程没有提供通用自动生成器。外观、预测成功、发生运动和任务完成是不同里程碑；缺哪一项就报告哪一项。

## 提交材料

资产版本和取得权限、URDF检查输出、填好的部件映射、静态多视角、相机外参、完整碰撞覆盖说明；如已接入闭环，另附真实请求与轨迹。未经运行的模板检查不能写成仿真或导航验收通过。
