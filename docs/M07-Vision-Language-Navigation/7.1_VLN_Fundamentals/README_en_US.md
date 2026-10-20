# 7.1 Introduction to Vision-Language Navigation

[English](README_en_US.md) | [中文](README_zh_CN.md)

“Locate the refrigerator in the room and stop beside it.” This instruction gives a robot neither map coordinates nor a turn-by-turn route. The robot must look for the target in its camera images, choose how to approach it, and decide when to stop. This chapter uses that task to introduce vision-language navigation.

## Learning Objectives and Prerequisites

- Explain the roles of the instruction, camera images, navigation decisions, and motion feedback.
- Distinguish the current image from visual history.
- Describe how maps, learned models, and controllers can work together.
- Define a goal, operating conditions, and measurable completion criteria for an application.

No ROS 2, SLAM, or Nav2 installation is required for this introduction. Chapter 7.2 uses arrays, probability, and basic trigonometry; chapter 7.3 assumes basic Linux and Python skills.

## 7.1.1 What Does One Instruction Require?

![Finding a refrigerator and choosing an approach](assets/images/01-find-refrigerator.jpg)

*Figure 7.1-1. Route choices and stopping areas in an indoor refrigerator search.*

When entering an unfamiliar room, a person might look for an opening toward the kitchen, recognize a refrigerator among other appliances, approach across clear floor, and choose a place to stand. This combines knowledge of object appearance, observations of the room, and a judgment about where there is enough space to move.

We will use the same instruction throughout:

```text
Locate the refrigerator in the room and stop beside it.
```

It contains four requirements:

| Expression | What the system must understand | Effect on navigation |
| --- | --- | --- |
| Locate | Establish where the target is; search if it is not visible | Turn or move to a new viewpoint |
| the refrigerator | Recognize a refrigerator and identify the intended instance | Distinguish it from similar cabinets or doors |
| in the room | Interpret the search area | Relate observations acquired along the way |
| stop beside it | Choose an accessible stopping place near the refrigerator and end motion | Consider both proximity and room for the robot |

“Beside” gives neither an exact distance nor a particular side. An application needs a stopping region that reflects the available space and the user's intent, followed by an evaluation of whether the robot reached it. “Stop on the clear floor in front of the refrigerator without blocking its door” is more specific. The video retains the original wording, so its interpretation also retains this ambiguity.

**Think about it:** How would you clarify the instruction if the room contained two refrigerators? Which places remain suitable if one side is against a wall and the other is cluttered?

## 7.1.2 What Is Vision-Language Navigation?

**Vision-and-Language Navigation (VLN)** studies how an agent combines a natural-language instruction with visual observations acquired along a route to choose actions and complete a movement task. The agent may be a physical robot or a simulated vehicle in MuJoCo. Both need to observe, decide how to move, and observe again after moving.

A compact description of one decision is:

$$
a_t=\pi_\theta(I,O_{\leq t}).
$$

$I$ is the instruction; $O_{\leq t}$ is the set of camera observations available by decision $t$; $\pi_\theta$ is the trained decision model with parameters $\theta$; and $a_t$ is its action representation. Depending on the method, that representation might be a category such as “turn left” or a short local path.

LightNav expresses its outputs as **tokens**, symbols in the model's vocabulary. A token can represent a text fragment or a specially defined pointing or action code. The model generates pointing symbols and action codes, and a decoder turns the action codes into ten local waypoints. A controller uses the path and the robot's current pose to calculate linear and angular velocity. The simulator then updates position and heading. Chapter 7.2 explains prediction and decoding; chapter 7.3 shows how to run the integrated system.

### VLN, VLM, and VLA Describe Different Things

| Term | Meaning | Role here |
| --- | --- | --- |
| VLM: Vision-Language Model | A model that processes images and text together | Provides a basis for recognizing objects and interpreting spatial relationships and instructions |
| VLA: Vision-Language-Action model | A model that produces an action representation conditioned on vision and language | LightNav encodes navigation actions as tokens it can generate |
| VLN: Vision-and-Language Navigation | A task that uses language and vision to guide movement | The problem the complete system addresses |

For the refrigerator task, a VLM can support the judgment “the refrigerator is on the right.” A VLA additionally produces an action representation. A VLN system must execute actions, acquire new images, and decide when to finish. These provide separate opportunities to evaluate understanding, action generation, and navigation.

### Three Common Types of Navigation Instruction

| Type | Example | What the robot must do |
| --- | --- | --- |
| Follow a route | “Go through the door, turn left, and stop at the second window.” | Recognize landmarks, follow the sequence, and track route progress |
| Find an object | “Find the refrigerator and stop beside it.” | Search, choose an approach, and select a stopping place |
| Follow a moving target | “Follow the person wearing blue.” | Maintain the target's identity and adjust as the person moves |

This chapter focuses on finding an object. Route following places more emphasis on matching ordered steps to landmarks. Following a person also requires tracking a changing target. Each places different demands on observation history and completion conditions.

## 7.1.3 Grounding Language in the Environment

Coordinate-based navigation can accept a target pose such as $(5.5\,\mathrm{m},15.6\,\mathrm{m},0\,\mathrm{rad})$. In a given map, these numbers specify a position and heading. Localization, planning, and control work together to reach it.

“Find the refrigerator” instead describes an object. The system must identify which visible object matches the description, or seek another viewpoint if it cannot see one. **Grounding** means linking language to objects, regions, or locations in the environment. Associating “refrigerator” with a gray appliance in an image is image-level grounding. Identifying accessible floor beside it brings the system closer to a usable movement goal.

### How Maps and Learned Models Can Work Together

One approach associates objects with a map and translates language into a goal region. The system might locate a refrigerator in a semantic map and call a planner to approach nearby clear floor. Learned models can help recognize objects or interpret the instruction.

Another approach predicts actions or local paths mainly from images and language. LightNav follows this approach: it uses current and past visual cues to propose a reference path, then makes another decision from new observations. Its input interface does not require a map annotated with refrigerator coordinates.

These approaches can be combined. A map represents environmental information; a learned model processes information. A system can use both. Here we focus on image-driven LightNav and the execution layer that handles coordinates and collisions.

In the experiments, MuJoCo supplies the robot's actual pose for path transformation, camera rendering, and logs. Scene geometry supports collision checking. Refrigerator evaluation coordinates are not passed to the model as a hidden hint. **The model proposes a path from images and language; the integration layer connects that proposal to simulated motion.**

## 7.1.4 Using Earlier Images and Moving to See More

![Changing viewpoints and visual history](assets/images/03-observe-remember-act.jpg)

*Figure 7.1-2. Changes in position and heading reveal new views, while earlier images provide clues for later decisions.*

### Something Missing from the Current View May Have Appeared Earlier

A camera captures only part of the space in front of it. On entering a room, the robot may see a sofa. At the doorway it may discover a refrigerator behind the door. After another turn, the refrigerator may leave the image again. The current image therefore cannot describe the whole room or everything the robot has already seen. This is **partial observability**: current sensor output reveals only part of the environment's state.

**Visual history** means the camera images collected, in order, during this navigation task. “A sofa at the start,” “a refrigerator at the doorway,” and “a wall after turning” form a visual history. These observations come from the current run; images used to train the model are a different source of information.

The system saves observations and selects some of them to accompany the current image and instruction. If the refrigerator temporarily leaves view, an earlier image containing it may still help the next decision. Consecutive images also provide clues about how the viewpoint changed.

Whether the model uses those clues correctly must be checked in its outputs. Saving images does not automatically produce an accurate inventory of every searched region. Chapter 7.2 explains how LightNav selects history and processes recent and older views differently.

Order matters. An earlier image can be useful; presenting it as a newly captured observation or scrambling the sequence supplies incorrect time cues. Similar images captured while the robot is stationary are normal. They must be distinguished from submitting the same capture record twice.

### An Action Can Reveal the Environment

If a sofa blocks the area behind a doorway, the current image may not reveal whether a refrigerator is there. Moving to a viewpoint that exposes the opening changes both the robot's position and the information in its next image.

A turn can therefore be evaluated in two ways:

1. Did it help the robot approach the target?
2. Did it expose a previously occluded area?

The second question requires comparing actual images before and after motion. A predicted turn alone is not evidence of useful exploration.

### Seeing a Target and Fitting Through an Opening Are Separate Questions

Raising a camera may reveal the refrigerator above a sofa. Passing through a doorway depends on vehicle width, heading, and obstacles. The virtual camera here is about 1 m high, and the DM-H65 body is approximately 0.49 m wide. Collision checks also cover wheels and caster envelopes.

When adapting a chassis, check separately whether the image provides useful target cues, whether the whole robot fits along the path, and whether the controller can track it.

### What Exactly Is Fed Back in the Loop?

There are two feedback streams. The **model** receives new camera images after movement and uses them for its next prediction. The **controller** continually receives the robot's current position and heading to correct tracking error. They run at different rates, but both must correspond to the current simulation state.

For example, the model proposes an approach to the doorway. The controller moves the robot, the camera renders a closer view, and the next prediction uses that view to adjust the approach. Decisions affect movement, movement changes observations, and observations affect the next decision: this is the closed loop.

LightNav inference may take seconds, while the controller updates every few tens of milliseconds. The system can keep following an existing path during inference while that path remains valid. It stops when the path is exhausted, expires, or becomes unsafe. Chapter 7.3 introduces the running demo that uses this asynchronous arrangement.

## 7.1.5 Where Can VLN Be Useful?

![Applications of vision-language navigation](assets/images/02-vln-applications.jpg)

*Figure 7.1-3. Applications in home assistance, delivery, search, inspection, and assisted mobility.*

| Setting | Example request | Deployment questions |
| --- | --- | --- |
| Home assistance | “Wait beside the refrigerator in the kitchen.” | Ambiguous objects, furniture occlusion, children, pets, and privacy |
| Hotel delivery | “Go to the service desk beside the elevator.” | Similar layouts, access control, elevator interaction, and people |
| Warehouse search | “Find the blue rack near the loading area.” | Attribute matching, aisle width, and changing inventory locations |
| Facility inspection | “Go to the electrical cabinet on the right and take a photo.” | Arrival pose, camera angle, authorization, and failure recovery |
| Assisted mobility | “Take me to the clear space on the right of the entrance.” | Comfort, reliable stopping, and takeover |

Natural language lets users express intent without memorizing map IDs, coordinates, or a complex interface. It also introduces ambiguity, and visual recognition can fail. Application design should establish whether the camera can observe the goal, whether the instruction can become measurable conditions, how failure is handled, and how the user stops the task.

Work backward from the need. Fixed-location delivery may be served by a destination list and a planner. Frequent new object descriptions make language understanding and open-category recognition more valuable. VLN is useful when it reduces a concrete user burden and can handle the resulting uncertainty.

**Application exercise:** Choose a setting above. Write an ordinary request, an ambiguous request, and one the current camera cannot support. Explain which conditions need clarification.

## 7.1.6 Evaluating “Found It and Stopped Beside It”

Distinguish four outcomes before evaluating a run:

| Outcome | Evidence to check |
| --- | --- |
| Valid model prediction | Correct request/response association, decodable tokens, and finite path values |
| Closed-loop operation | Predictions affect motion, which produces new observations and later predictions |
| Target approach | Actual movement reduces the independently measured target distance |
| Task success | Preregistered goal relationship, termination, stable stopping, and collision conditions all hold |

A model STOP asks to end the task; a user STOP cancels it. An evaluator must also inspect the stopping position and movement record. If the refrigerator task has no registered evaluation region, report observed approach and stopping without borrowing another target's coordinates to assign a score.

### Success Rate: SR

For $N$ independent trials, let $S_i=1$ for success and $S_i=0$ for failure:

$$
\mathrm{SR}=\frac{1}{N}\sum_{i=1}^{N}S_i.
$$

Six successes in ten trials give an SR of 0.6. Define start states, goals, budgets, and scoring rules before testing, then report every result. A selected successful video cannot estimate overall success rate.

### Success Weighted by Path Length: SPL

A successful robot may still take a long detour. SPL combines success and route efficiency:

$$
\mathrm{SPL}=\frac{1}{N}\sum_{i=1}^{N}S_i\frac{L_i}{\max(P_i,L_i)}.
$$

$L_i$ is the shortest feasible path from the start to a valid goal region, and $P_i$ is actual travel distance, in the same units. A successful 3 m route with a 2 m shortest feasible path contributes about 0.667. A failure contributes zero. The shortest route must account for furniture and passages, rather than use a straight line through obstacles.

Benchmarks define different arrival distances, target regions, and stopping conditions. Check those conditions before comparing SR or SPL.

## Review and Self-Check

The refrigerator task connects language goals, current images, visual history, model decisions, and motion feedback. The model proposes actions, the controller tracks a reference using pose feedback, and new images inform later decisions. Prediction, closed-loop operation, approach, and task completion are evaluated separately.

1. What does visual history contain, and how does it differ from training data and a map?
2. Why does detecting the refrigerator still leave the problem of choosing an accessible stopping place?
3. What feedback reaches the model, and what feedback reaches the controller?
4. How could images before and after a turn show whether it reduced occlusion?
5. What SPL contribution does a successful 4 m route make if the shortest feasible route is 2 m?
6. Why record model STOP, user STOP, and task success separately?

**Deliverables:** Break down the refrigerator instruction, describe a possible observation sequence, and define operating conditions and evaluation rules for one application.

## Further Reading

- [R2R paper](https://arxiv.org/abs/1711.07280): natural-language route instructions and navigation.
- [Habitat evaluation recommendations](https://arxiv.org/abs/1807.06757): success and path-efficiency measures.
- [Official VLN-CE implementation](https://github.com/jacobkrantz/VLN-CE): navigation in continuous environments.
- [LightNav-0 paper](https://arxiv.org/html/2608.30935v2): the algorithm used in this module.

[Back to M7](../README.md) · [7.2 Understanding LightNav-0](../7.2_Semantic_Grounding_and_Learned_VLN/README_en_US.md)
