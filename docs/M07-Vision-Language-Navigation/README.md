# M7 Vision-Language Navigation: From LightNav to a Jetson Demo

[English](README.md) | [中文](README_zh_CN.md)

> Locate the refrigerator in the room and stop beside it.

Learn how language and camera observations become a local path, then run a real LightNav-0 GPU model with a DM-H65 chassis in MuJoCo on AGX Orin. Follow one task from VLN concepts to algorithm interpretation and a running web demo.

![LightNav refrigerator navigation demo](7.3_From_Navigation_Prediction_to_Closed_Loop_Control/assets/demo.gif)

*The chassis observes the room, follows predicted paths, and approaches the refrigerator.*

## Chapters

| Chapter | Main question | Deliverable |
| --- | --- | --- |
| [7.1 Introduction to Vision-Language Navigation](7.1_VLN_Fundamentals/README_en_US.md) | How do language, observations, actions, and feedback form a navigation task? | Instruction analysis and evaluation criteria |
| [7.2 Understanding LightNav-0](7.2_Semantic_Grounding_and_Learned_VLN/README_en_US.md) | How do SlowFast, APOS/OPOS, and RVQ produce ten waypoints? | An explanation of one prediction |
| [7.3 Running the LightNav Navigation Demo on Jetson](7.3_From_Navigation_Prediction_to_Closed_Loop_Control/README_en_US.md) | How do you start services, open the page, and run an instruction? | A recorded demo session |

Start with concepts, continue with the algorithm, then run the prepared demo. ROS 2, SLAM, and Nav2 are not installation prerequisites for this MuJoCo exercise. The three chapters, lab guide, advanced reading, and code entry are available in English and Chinese. Historical records retain their original language.

## Directory Layout

```text
M07-Vision-Language-Navigation/
├── README.md / README_zh_CN.md
├── 7.1_VLN_Fundamentals/          # English/Chinese text and images
├── 7.2_Semantic_Grounding_and_Learned_VLN/
│   ├── README_en_US.md / README_zh_CN.md
│   ├── ADVANCED_READING.md / ADVANCED_READING_zh_CN.md
│   └── assets/                  # Images and teaching animations
├── 7.3_From_Navigation_Prediction_to_Closed_Loop_Control/
│   ├── README_en_US.md / README_zh_CN.md
│   ├── LAB_GUIDE.md / LAB_GUIDE_zh_CN.md
│   └── assets/demo.gif
├── code/                        # Source overlay, configs, locks, tools, examples
├── assets/                      # Shared and archived materials
└── SHA256SUMS.txt
```

## Code and Practice

- **Code:** Begin at [code/README.md](code/README.md) for the package layout, fixed versions, checks, and preparation tools.
- **Prepared device:** Follow [7.3](7.3_From_Navigation_Prediction_to_Closed_Loop_Control/README_en_US.md) and [Lab Guide, Route A](7.3_From_Navigation_Prediction_to_Closed_Loop_Control/LAB_GUIDE.md#route-a-use-an-already-deployed-device), checking current status before reusing services.
- **New device:** Follow [Route B](7.3_From_Navigation_Prediction_to_Closed_Loop_Control/LAB_GUIDE.md#route-b-prepare-a-new-device) to prepare pinned source, two environments, the model, scene, and external CAD bundle.
- **Your own chassis:** Use the [custom-chassis materials](code/examples/custom-chassis/README.md) to inspect components and build an independent preview before backend adaptation.
- **Without hardware:** Watch the demo and analyze [archived real records](code/evidence/README.md).
- **More algorithm detail:** Consult [Advanced Reading](7.2_Semantic_Grounding_and_Learned_VLN/ADVANCED_READING.md).

## Progress and Experimental Results

**Course materials:** English and Chinese chapters, supporting practice guides, code, and a running-demo recording are organized here.

**Verified implementation:** Existing evidence shows real LightNav GPU inference and continuous chassis motion in simulation. The demo uses simplified differential-drive kinematics, geometric lookahead tracking, and fast collision checks. It records original model paths, limited local repairs, and actual trajectories separately. Physical tire/caster dynamics and braking calibration are outside this demo.

**Refrigerator case:** The supplied video is labeled 3× speed and shows 17 predictions, approach to the appliance, and model STOP. It is a custom task without registered independent success scoring. The video ending is therefore an observation of behavior, not a measured success rate.

**Archived sample:** The earlier doorframe record has five predictions and ends in unsafe-path rejection, with strict success false. It is distinct from both the refrigerator video and a separate eight-prediction refrigerator failure run.

**External materials:** Weights, ARM wheels, and the instructor CAD bundle must be acquired separately. The CAD bundle has no public download URL, so new-device reproduction depends on instructor access. See the [Lab Guide](7.3_From_Navigation_Prediction_to_Closed_Loop_Control/LAB_GUIDE.md) and [checksums](SHA256SUMS.txt).
