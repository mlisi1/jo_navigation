# jo_navigation

[![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-34C759?style=flat-square&logo=ros)](https://docs.ros.org/)
[![Nav2](https://img.shields.io/badge/Nav2-Latest-4285F4?style=flat-square&logo=ros)](https://navigation.ros.org/)
[![Ubuntu 24.04](https://img.shields.io/badge/Ubuntu-24.04%20LTS-E95420?style=flat-square&logo=ubuntu)](https://ubuntu.com/)

ROS 2 navigation and localization package for the Jo tracked robot platform. Provides localization via sensor fusion (EKF) and autonomous navigation using the Nav2 stack, with support for both local (odometry-based) and global (GPS-based) operation.


<div align="center">
  <img src="res/navigation.gif" alt="Navigation Stack" height="500"/>
</div>


## Package Contents

- **Launch files** — for localization and navigation in local and global modalities
- **EKF configurations** — tuned for odometry-only and GPS-fused localization
- **Nav2 configuration** — costmap, planner, and controller parameters for the tracked platform
- **RViz configs** — pre-configured layouts for monitoring the perception and navigation stack



## Localization

The localization stack is responsible for estimating the robot's pose and publishing the relevant TF transforms. Two modes are available depending on the available sensor inputs.

> **Note:** GLIM (the SLAM system used in Jo) does not publish on `/tf` by default, so the localization launch files are required to obtain a stable fixed frame in RViz and for Nav2 to function correctly.

### Local odometry

Runs a single EKF node that fuses wheel odometry and IMU data to estimate the `odom` → `base_link` transform. 

```bash
ros2 launch jo_navigation localization.launch.py
```

Once running, `odom` can be used as the fixed frame in RViz.

### Global localization (GPS-fused)

Runs two EKF instances in tandem:

1. **Local EKF** — estimates `odom` → `base_link` (identical to the local odometry launch)
2. **Global EKF** — fuses GPS data to estimate `map` → `odom`, providing a globally-referenced pose

```bash
ros2 launch jo_navigation localization_gps.launch.py 
```

Heading: both EKFs take yaw from GLIM (the IMU gives only the yaw rate: the Xsens magnetometer yaw is
disturbed on the robot). GLIM's `odom` frame has an arbitrary heading, so before every GPS mission run
the **heading alignment** (`heading_align` alias in jo-zotac):

```bash
ros2 run jo_navigation heading_alignment_node.py --ros-args \
    --params-file $(ros2 pkg prefix jo_navigation)/share/jo_navigation/config/heading_alignment.yaml
```

Enter starts a straight 10 m drive at 0.3 m/s (no obstacle avoidance: keep the RC/e-stop in hand, base in
CAN mode), whatever the GPS quality; the node fits the GPS track, compares it with GLIM's heading, shows the
result with any warnings, and Enter applies it to navsat_transform (`~/datum`). With warnings about the GPS
track (not straight, distance mismatch) answer `n` and run it again with a longer drive
(`-p calib_distance_m:=20.0`). Then start the mission.

## Navigation

The navigation stack is built on Nav2 and requires a localization source to be running beforehand. Two configurations mirror the two localization modalities.

### Local navigation

Uses the `odom` frame as the planning reference. Suitable for navigation in a known local area without a global map.

**Requires:** [Local odometry](#local-odometry) to be running.

```bash
ros2 launch jo_navigation navigation_local.launch.py rviz:=true
```

This launches:
- Nav2 stack (planner, controller, costmap nodes)
- RViz configured to display the perception stack, local costmap, and planned path

To command the robot, use the **2D Goal Pose** tool in RViz and click a target on the map. Set the fixed frame to `odom`.

**Demo circle**: a circle of waypoints around the robot (`demo_circle <radius_m> <points>` alias in jo-zotac).
The robot's current pose is the centre; the first waypoint is straight ahead on the circumference, the others
evenly spaced counter-clockwise, each oriented along the tangent (Nav2 `follow_waypoints`, `odom` frame).

```bash
ros2 run jo_navigation demo_circle.py --ros-args -p radius_m:=5.0 -p points:=15
# optional: -p clockwise:=true, -p close_loop:=true (the first waypoint again at the end)
```

Speed: the velocity smoother caps the local navigation at 0.7 m/s (`nav2_local.yaml`).

### Global navigation

Uses the `map` frame published by the global EKF. Supports navigation over larger areas with GPS-anchored pose estimates.

**Requires:** [Global localization](#global-localization-gps-fused) to be running.

```bash
ros2 launch jo_navigation navigation_gps.launch.py rviz:=true
```

This is a modified version of the local navigation configuration adapted to consume the `map` frame and the dual-EKF odometry sources.


## Additional Resources

- [Nav2 Documentation](https://navigation.ros.org/)
- [robot_localization Documentation](https://docs.ros.org/en/melodic/api/robot_localization/html/index.html)
- [ROS 2 TF2](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Tf2.html)
- [jo_sim](https://github.com/mlisi1/jo_sim.git)
- [jo_zotac](https://github.com/mlisi1/jo-zotac.git)

## GLIM (glim_ros2@dev)

The localization launch files require the [mlisi1/glim_ros2](https://github.com/mlisi1/glim_ros2) `dev`
interface (and koide3/glim >= 1a197c9, which needs gtsam_points newer than 1.2.2: build it from source):

- the EKFs fuse `glim_ros/odometry/local` (optimized odometry, expressed in `base_link`);
- `localization.launch.py`: `map_odom_mode: external` (EKF owns `odom -> base_link`, GLIM owns `map -> odom`);
- `localization_gps.launch.py`: `publish_tf: false` (EKFs and navsat_transform own the tree);
- maps are saved to `/saved_maps` (`ros2 service call /glim_ros/save_map std_srvs/srv/Trigger`);
- `glim_view:=false` drops the viewer through the `extension_modules` parameter;
- `glim_config:=<dir>`: GLIM config directory (`glim_config_bunker` on the robot, `glim_config_bunker_sim` in jo_sim);
- `glim_anchor_frame:=imu|base`: origin of GLIM's map/odom frames, the initial IMU pose (default, upstream behavior) or the initial base pose (x, y, z, yaw; jo_sim uses `base`).

Visual odometry is skipped with a message when RTAB-Map (`rtabmap_odom`) is not installed.
