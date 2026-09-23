# detection_lane_opencv

Classical (non-ML) lane detection for [`car_sim`](https://github.com/Bishop-11/car_sim):
HSV color-thresholds the white road-boundary lines from the front camera
feed, then reprojects them onto the ground plane using the camera's known
intrinsics/extrinsics to estimate where the car sits within the lane, its
heading relative to the road, and the road's curvature ahead. The yellow
centerline is deliberately never used.

Because the camera's exact geometry is known (published by `car_sim`, not
calibrated), the reprojection is an exact closed-form calculation — the
inverse of the pinhole projection, intersected with the ground plane — not
an approximation.

## Pipeline

```
/car/camera/image_raw ──▶ color_segmentation_node ──▶ /detection/segmentation_image ──┐
                            (HSV threshold, white                                        │
                             boundary lines only)                                        ▼
/car/camera/camera_info ──────────────────────────────────▶ lane_geometry_node ──▶ /detection/output
/car/camera/extrinsics  ──────────────────────────────────▶ (reprojects mask to      (JSON string)
                                                               ground plane, fits
                                                               left/right edges,
                                                               derives position/
                                                               heading/curvature)
```

Two independent nodes/scripts, the same separation of concerns as
`car_sim`'s `road.py`/`render.py` split:
- **`color_segmentation_node.py`** only knows about pixels and colors —
  never camera geometry or world coordinates.
- **`lane_geometry_node.py`** only knows about a binary mask + camera
  geometry — never color thresholds.

`/detection/segmentation_image` is dual-purpose: it's the exact mask
`lane_geometry_node` consumes, and it's directly viewable (in `rqt_image_view`
or similar) as a debug image — white pixels are detections.

## Topics

| Name | Type | Direction | Notes |
|---|---|---|---|
| `/car/camera/image_raw` | `sensor_msgs/Image` | sub (color_segmentation_node) | from `car_sim` |
| `/car/camera/camera_info` | `sensor_msgs/CameraInfo` | sub (lane_geometry_node) | from `car_sim`; intrinsics |
| `/car/camera/extrinsics` | `geometry_msgs/PoseStamped` | sub (lane_geometry_node) | from `car_sim`; camera pose in the car's own body frame |
| `/detection/segmentation_image` | `sensor_msgs/Image` (mono8) | pub (color_segmentation_node) / sub (lane_geometry_node) | binary mask, white-boundary pixels only |
| `/detection/output` | `std_msgs/String` (JSON) | pub (lane_geometry_node) | see below |

`/detection/output` payload:
```json
{"stamp": 1234.5, "position": 0.12, "heading": -0.03, "curvature_radius": 84.2}
```
- **`position`** — car's lateral position within the lane, `[-1, 1]`:
  `-1` = at the left boundary, `0` = centered, `+1` = at the right boundary.
- **`heading`** — road tangent angle relative to the car's own forward axis,
  radians. Positive = the road bends toward the car's left from here.
- **`curvature_radius`** — signed road curvature ahead, meters. Positive =
  curving right, negative = curving left; large magnitude ≈ straight.

Not published (skipped that tick) when either boundary doesn't have enough
confidently-detected points to fit reliably — a downstream controller then
just sees the last valid value rather than a fresh noisy one.

## How the geometry works (`lane_geometry_node.py`)

Everything is computed in the **car's own body frame** (x-forward, y-left,
z-up, origin at the car) — there's no global localization assumed, matching
what a real onboard perception stack actually has access to.

1. For every white mask pixel `(u, v)`, build its camera-frame ray direction
   from the intrinsics: `((u-cx)/fx, (v-cy)/fy, 1)`.
2. Rotate that ray into the car's body frame using the extrinsics
   (`R_body_from_cam`, published by `car_sim`).
3. Intersect the ray with the ground plane (body-frame `z = 0`) to get an
   exact `(X, Y)` point in meters, in the car's own frame.
4. **Confidence gating**: reject rays that don't point meaningfully downward
   (near/above the horizon — a tiny pixel error there implies a huge
   distance error, a fundamental limitation of monocular reprojection, not a
   bug) and reject any point beyond `max_range_m`. Together these keep only
   the near/mid-field, where the reprojection is numerically well-conditioned.
5. Split the remaining points into left (`Y > 0`) / right (`Y < 0`) boundary
   clusters, and fit a quadratic `Y = aX² + bX + c` to each
   (`numpy.polyfit`). If either side has fewer than `min_points_per_side`
   points, skip publishing this tick.
6. Evaluate both fits at `X = 0` (the car's own position) for `position`;
   average their slope there for `heading`; average their curvature
   coefficient for `curvature_radius` (sign-flipped to match the
   positive-right/negative-left convention above).

## Parameters

| Node | Param | Meaning |
|---|---|---|
| `color_segmentation_node` | `white_s_max` | max HSV saturation still counted as "white" (default 60) |
| `color_segmentation_node` | `white_v_min` | min HSV value/brightness counted as "white" (default 180) |
| `lane_geometry_node` | `max_range_m` | reject reprojected points beyond this distance (default 25.0) |
| `lane_geometry_node` | `min_points_per_side` | minimum detected points per boundary to trust a fit (default 15) |

## Running it

Requires `car_sim` running first (either `sim_only.launch.py` for automatic
control, or `manual_play.launch.py` to drive it yourself):
```
ros2 launch detection_lane_opencv detection.launch.py
```

## Repo layout

Own repo in the `lane_drive_ws/src/` multi-repo workspace, alongside
`car_sim`. Consumes only `car_sim`'s published topics (image, camera_info,
extrinsics) and outputs `/detection/output` for a downstream controller
(`car_pid_control`, `car_rl_control`, ...) to consume — no direct code
dependency on `car_sim` in either direction.
