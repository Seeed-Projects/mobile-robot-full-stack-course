# Module Map

## Dependency Overview

| Module | Prerequisites | Main Deliverable |
| --- | --- | --- |
| M1 | None | Reproducible J501 development baseline |
| M2 | M1 | Camera capture and calibration toolkit |
| M3 | M1 | LiDAR and multi-sensor synchronized pipeline |
| M4 | M2 | Accelerated vision inference demos |
| M5 | M2 + M3 | Mapping, reconstruction, and semantic spatial outputs |
| M6 | M3 + M5 | Navigation and planning demo |
| M7 | M4 + M6 | Language-grounded navigation behavior |
| M8 | M4 + M6 | PTZ and active vision control loop |
| M9 | M1 + M4 | Vision-guided manipulation pipeline |
| M10 | M1 to M9 | Integrated end-to-end system demos |
| M11 | M1 to M10 | Deployment-ready engineering assets |

## Progression Logic

- Perception modules create the sensor and inference pipeline.
- Cognition modules create the robot's world model.
- Decision modules make motion and semantic planning possible.
- Actuation modules close the loop with PTZ and manipulation.
- Deployment turns the best demos into maintainable systems.

