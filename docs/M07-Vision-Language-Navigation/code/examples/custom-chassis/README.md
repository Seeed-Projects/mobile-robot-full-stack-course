# Custom Chassis: From CAD Inspection to a MuJoCo Preview

[English](README.md) | [中文](README_zh_CN.md)

These course materials support independent inspection and preview; they do not install into the running project or change its controller. Start with [chapter 7.3](../../../7.3_From_Navigation_Prediction_to_Closed_Loop_Control/README_en_US.md).

## 1. Distinguish Three Milestones

| Stage | Output | Still to establish |
| --- | --- | --- |
| CAD inspection | Components, joint tree, references, unit records, and SHA256 | Scene collisions and dynamics |
| MuJoCo preview | Appearance, scale, coordinates, and camera | Model-driven navigation |
| Room closed loop | Backend, real input images, predictions, execution, and new observations | Stable success rate and physical-vehicle agreement |

The fixed DM-H65 adapter expects 43 visual links and two drive wheels. Substituting another URDF into its configuration is not a general adaptation procedure.

## 2. Inspect a Real URDF Without Modifying It

```bash
python3 docs/M07-Vision-Language-Navigation/code/examples/custom-chassis/inspect_urdf.py \
  /path/to/export/robot.urdf \
  --package your_robot_description=/path/to/your_robot_description
```

Relative paths resolve against the URDF directory. Explicitly map package URIs; the tool does not download resources. It checks for a single-root joint tree, duplicate names, cycles, missing meshes, and file hashes, then prints results without writing source assets.

`valid_inventory=true` confirms reference/inventory checks only. Check dimensions, degrees of freedom, collision coverage, dynamics, and navigation separately. Verify STL units against wheel and body dimensions; the tool does not infer meters versus millimeters.

## 3. Map Components

Copy [part-mapping.example.json](part-mapping.example.json) into your own workspace and fill in actual part names, axle centers, casters, dimensions, and camera. It is a design-record template, not direct input for the fixed loader.

Preserve the original CAD export and record whether meshes have already been transformed into the root frame. Track visual and collision sources separately. Drive-wheel, caster-swivel, and caster-roll axes cannot be inferred reliably from opaque filenames.

## 4. Preview an Independent MJCF

[preview.xml](preview.xml) is a static primitive template with two main wheels and four casters. It has no real DM-H65 meshes, drive, or control. Use it to learn part relationships, then replace it with the authorized CAD geometry.

On a computer with MuJoCo installed:

```bash
python -m mujoco.viewer --mjcf=/absolute/path/to/preview.xml
```

On headless Jetson, render independently with the existing simulation environment, without starting the model or online demo:

```bash
MUJOCO_GL=egl /path/to/mujoco-env/bin/python - <<'PYCODE'
import mujoco
from PIL import Image
model = mujoco.MjModel.from_xml_path('/absolute/path/to/preview.xml')
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)
with mujoco.Renderer(model, height=270, width=480) as renderer:
    renderer.update_scene(data, camera='ego')
    Image.fromarray(renderer.render()).save('/tmp/my-chassis-ego.png')
PYCODE
```

Clear floor in front of the camera is expected for this template. Declare authorized meshes in `asset`, add `type="mesh"` visual geoms, disable their collision participation, and cover the actual shape with separate collision geometry. Then check the room model and whether its camera can observe the task target.

## 5. Integrate a Backend in a Project Copy

Implement a new adapter rather than removing the fixed DM-H65 checks. Refer to the overlay's `robots/dm_h65.py` and `sim_demo_runtime.py` for these responsibilities:

- Model construction: merge scene and chassis with stable root body/joint, collision-geom, and camera names.
- State: read root pose, actual velocity, camera extrinsics, and contacts from the same MuJoCo instance.
- Kinematics: X forward, Y left, counterclockwise positive yaw; call `mj_forward` after updating state and render new observations from that state.
- Geometry: cover the entire body and steering-part envelope, with continuous-path prechecks and runtime checks.
- Lifecycle: STOP, RESET, cancellation generations, stale-result rejection, path TTL, and disconnection.
- Display and logs: distinguish raw model input/output, execution reference, and actual state.

The course does not provide a universal CAD-to-backend generator. Appearance, valid predictions, movement, and task completion are separate milestones; report which ones have actually been reached.

## Deliverables

Record asset version and usage rights, URDF inspection output, completed part mapping, static views, camera extrinsics, and full collision coverage. Add real requests and trajectories if closed-loop integration is complete. A template that has not been run is not simulation or navigation validation.
