# 2.4 Multi-Camera BEV Stitching and Surround-View Perception Fundamentals

### Hardware Preparation

- reComputer J501 with four 3G GMSL cameras (198° horizontal field of view)
- Four A4 chessboards
- A 3D-printed cross mount (four cameras spaced 90° apart, lenses tilted downward by 22°, installation height: 10 cm)

### Prerequisites

#### Why Do We Need BEV?

BEV (Bird's Eye View) transforms what cameras see into a top-down map. A conventional camera produces a perspective image in which nearby objects appear larger and distant objects appear smaller, making it difficult to determine true spatial relationships directly. BEV uses camera-calibration data to project road-surface content onto a common plane, so vehicles, pedestrians, and obstacles appear as if they were placed on a map. This representation is easier for people to interpret and for machines to use when measuring distances, estimating relative positions, and planning motion. Whether the application is a 360° automotive surround-view system, automated parking, or robotic perception, the central purpose of BEV is to turn images into an understandable spatial map.

![image.png](./images/TTkLbruqUoKbKYxWq12cBXy4nff.png)

#### Camera Calibration

Camera calibration tells the computer how a camera observes the world.

For BEV, camera images must be projected accurately onto the ground plane. Without calibration, the computer sees only an ordinary image and does not know:

- How wide the lens is, or its focal length
- Where the image center is
- How much distortion the fisheye lens introduces
- Where the camera is installed
- Which direction the camera faces

It therefore cannot determine which real-world location corresponds to a pixel. For example, draw a 1 m × 1 m square on a sheet and photograph it with a fisheye camera. Perspective and distortion turn the square into a curved shape whose apparent size varies across the image. Calibration establishes the pixel-to-real-world correspondence and tells the system where each pixel lies on the ground.

BEV systems normally require two types of calibration:

- **Intrinsic calibration:** determines the camera's imaging characteristics—focal length, principal point, distortion, and related parameters—and is used to remove fisheye distortion.
- **Extrinsic calibration:** determines the camera's position and orientation relative to the vehicle or world—translation and rotation—and places all cameras in one coordinate system.

Only after both steps can the system flatten the front, rear, left, and right camera views onto the ground correctly and stitch them into a seamless 360° bird's-eye view. Otherwise, lane markings will fail to align, object positions will drift, and seams will remain visible.

#### Workflow

The surround-view perception pipeline contains six stages: four fisheye image inputs → camera calibration → fisheye undistortion → perspective transformation/BEV projection → image stitching and blending → 360° surround-view output. The following sections implement this pipeline step by step.

![image.png](./images/H4XRb1jdPowby2xschJc4jXSnZq.png)

#### BEV and AVM

BEV (Bird's Eye View) is a top-down spatial map, whereas AVM (Around View Monitor) is a human-facing 360° surround-view function. BEV focuses on transforming multiple camera views into one coordinate system so that the system can understand object positions, distances, and motion relationships. AVM builds on that representation by stitching and blending the images into an intuitive bird's-eye view for a driver. Put simply, BEV is responsible for understanding space, while AVM is responsible for presenting it. The former is oriented toward machine perception and the latter toward human-machine interaction; modern automotive surround-view systems are usually built on BEV technology.

![image.png](./images/RPTlb6E4LoyjI9x6mouc6dlPnHf.png)

### Step by Step: Multi-Camera BEV Stitching

This exercise builds a 360° surround view from four fisheye cameras. Before starting, complete Section 2.1 (GMSL Multi-Camera Integration) and Section 2.3 (Camera Calibration). Make sure all four cameras stream synchronously and that valid intrinsic parameters and distortion coefficients are available for each camera. The implementation follows this sequence: intrinsic calibration → extrinsics/homography → undistortion and BEV projection → stitching and blending → validation.

#### Step 1: Assemble the Cameras and Verify the Hardware

Mount four fisheye cameras on the 3D-printed cross at 90° intervals, tilt the lenses downward by 22°, and set the installation height to 10 cm. Connect all four cameras to the J501 GMSL expansion board with Fakra cables. Select the correct device tree as described in Section 2.1, restart the system, and verify that all four video nodes appear:

```bash
ls /dev/video*
# Expected: /dev/video0 through /dev/video3
```

For hardware-synchronized exposure across all four cameras, use the FSYNC configuration and the `gmsl4_start.sh` script described in Section 2.1.

#### Step 2: Start the Cameras and Verify Their Streams

Following the `gmsl4_start.sh` instructions in Section 2.1 and the `v4l2_camera` node configuration in Section 2.3, start a ROS 2 node for each camera and verify the image topics. Set the resolution according to the camera model:

```bash
ros2 topic list -t                    # Confirm that all four image_raw topics exist
ros2 topic hz /gmsl/cam0/image_raw    # Confirm that the streaming frame rate is stable
```

#### Step 3: Intrinsic Calibration for Fisheye Undistortion

Surround-view cameras commonly use fisheye lenses with a field of view of approximately 198°. A standard pinhole model cannot describe them accurately, so use a fisheye model to estimate the intrinsic matrix (K) and distortion coefficients (D). If you use the ROS 2 `camera_calibration` GUI, follow Section 2.3 but select a fisheye/equidistant distortion model. The equivalent OpenCV Kannala–Brandt implementation below supports offline batch processing of each camera's chessboard images. Capture 20–40 chessboard images at varied poses for every camera:

```python
import cv2
import numpy as np
import glob

pattern = (8, 6)          # Number of inner chessboard corners
square = 0.02             # Chessboard square size in meters

objp = np.zeros((pattern[0]*pattern[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:pattern[0], 0:pattern[1]].T.reshape(-1, 2) * square

objpoints, imgpoints = [], []
for f in sorted(glob.glob("calib/cam0/*.png")):
    img = cv2.imread(f)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ok, corners = cv2.findChessboardCorners(
        gray, pattern,
        cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not ok:
        continue
    corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6))
    objpoints.append(objp)
    imgpoints.append(corners)

h, w = gray.shape[:2]
K = np.zeros((3, 3))
D = np.zeros((4, 1))
rms, K, D, rvecs, tvecs = cv2.fisheye.calibrate(
    objpoints, imgpoints, (w, h), K, D,
    flags=cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC
          + cv2.fisheye.CALIB_CHECK_COND
          + cv2.fisheye.CALIB_FIX_SKEW)
print("cam0 RMS:", rms)
print("cam0 K =\n", K)
print("cam0 D =\n", D)
```

> Calibrate each camera separately and save its (K) and (D) as inputs to the later undistortion stage. Lower reprojection error (RMS) is better; as a general target, keep it below one pixel.

#### Step 4: Extrinsic Calibration and BEV Homography

Lay the four A4 chessboards flat on the ground around the cross mount, with each board fully visible to one camera. Starting from the undistorted images, solve a homography (H) from the ground plane to the image for each camera. Its inverse is the core of the inverse perspective mapping (IPM) that projects perspective images onto a common bird's-eye plane:

```python
# src_pts: pixel coordinates of ground-plane chessboard corners in the undistorted image
# dst_pts: target pixel locations of the same points in the world/ground frame
#          (meters converted to canvas pixels)
H, _ = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 3.0)

# Recommended world/vehicle frame: origin at the center of the cross mount,
# X axis forward and Y axis left.
# All four cameras must share this ground coordinate frame so that grids
# and lane markings align during stitching.
```

#### Step 5: BEV Projection (Undistortion + Inverse Perspective Mapping)

At runtime, first undistort the fisheye image and then use homography (H) to project it onto the BEV canvas. For real-time performance, combine undistortion and projection into a lookup table so that each frame needs only one remap operation:

```python
# 1) Fisheye undistortion: generate the mapping table once, offline
map1, map2 = cv2.fisheye.initUndistortRectifyMap(
    K, D, np.eye(3), K, (w, h), cv2.CV_16SC2)

# 2) Per-frame processing: undistort, then apply IPM onto the BEV canvas
undist = cv2.remap(frame, map1, map2, cv2.INTER_LINEAR)
bev = cv2.warpPerspective(undist, H, (bev_w, bev_h))
```

#### Step 6: Stitching and Blending

Place the four BEV images at their corresponding locations on one world-coordinate canvas to form the 360° surround view. Apply weighted blending in overlapping regions to remove seams and sudden brightness changes:

```python
canvas = np.zeros((canvas_h, canvas_w, 3), np.float32)
weight = np.zeros((canvas_h, canvas_w), np.float32)

for bev, H_world in zip(bevs, H_list):
    # Place each BEV image at its corresponding position on the world canvas
    warped = cv2.warpPerspective(bev, H_world, (canvas_w, canvas_h))
    # Valid contribution region for this view
    w = (cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY) > 0).astype(np.float32)
    canvas += warped.astype(np.float32) * w[:, :, None]
    weight += w

# Average overlapping contributions to obtain a smooth surround-view image
canvas = (canvas / np.maximum(weight[:, :, None], 1e-6)).astype(np.uint8)
cv2.imshow("360 AVM", canvas)
```

For smoother results, weight pixels in overlap regions according to their distance from the image boundary. This avoids seams introduced by simple averaging.

#### Step 7: Validation and Tuning

- **Seam alignment:** Inspect overlaps between adjacent cameras. Chessboard lines should remain continuous across seams. If there is visible misalignment, recalibrate the corresponding camera's extrinsics.
- **Distance accuracy:** Place an object of known dimensions at several locations. Verify that its scale is consistent in the BEV image to assess homography (H).
- **Brightness consistency:** If exposure or white balance differs substantially among cameras, lock exposure and white balance first (see the pre-calibration checks in Section 2.3), then normalize gain.
- **Real-time performance:** Prefer precomputed `remap` lookup tables over per-frame matrix calculations, especially at full resolution.

> After completing this section, the 360° surround-view image can serve as input to higher-level algorithms. When connecting lane/obstacle detection (M4) or visual SLAM (M3), configure the target node with the `camera_info` data from Section 2.3 together with the extrinsics/homographies from this section.

