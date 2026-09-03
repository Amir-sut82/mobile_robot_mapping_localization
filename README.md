# Mobile Robot Mapping, Localization, and A* Navigation

ROS 2 workspace for a differential-drive mobile robot in Gazebo. The workspace contains three related pipelines:

1. **Custom 2D SLAM** (`robot_slam`): estimates pose from odometry and LiDAR while building a log-odds occupancy grid.
2. **Map-based localization** (`map_publisher`): publishes a saved map and estimates pose with a wheel-odometry + IMU EKF followed by a particle filter (AMCL-style).
3. **Global planning** (`map_publisher`): inflates obstacles and computes an 8-connected A* path from the current localized pose to a requested goal.

The robot model, world, Gazebo bridge, RViz configurations, and low-level helper nodes are in `robot_description`.

## Repository layout

| Package | Purpose |
| --- | --- |
| `robot_description` | URDF, meshes, depot world, Gazebo/ROS bridge, RViz, motor and frame helpers |
| `robot_slam` | C++ motion model, scan matcher, occupancy grid, SLAM node, and map saver |
| `map_publisher` | Python map publisher, EKF, particle filter, A* planner, and launch files |

## Installation

The repository contains packages directly below its root. Clone it inside the `src` directory of a ROS 2 workspace:

```bash
source /opt/ros/<ros_distro>/setup.bash
mkdir -p ~/Localization_ws/src
cd ~/Localization_ws/src
git clone https://github.com/Amir-sut82/Localization.git
cd ~/Localization_ws

sudo apt update
sudo apt install -y \
  python3-rosdep python3-numpy python3-scipy python3-pil python3-yaml \
  ros-<ros_distro>-teleop-twist-keyboard

rosdep update
rosdep install --from-paths src --ignore-src --rosdistro <ros_distro> -r -y
colcon build --symlink-install
source install/setup.bash
```

Replace `<ros_distro>` with the installed distribution (for example, `jazzy` or `humble`). The Gazebo launch files also require the ROS-Gazebo packages supplied by that distribution (`ros_gz_sim`, `ros_gz_bridge`) and RViz 2.

## Running the custom SLAM pipeline

Terminal 1 starts Gazebo, the robot, bridge, wheel/IMU EKF, LiDAR frame conversion, custom SLAM, RViz, and the periodic map saver:

```bash
source ~/Localization_ws/install/setup.bash
ros2 launch robot_slam slam.launch.py
```

Drive the robot with keyboard teleoperation in Terminal 2:

```bash
source ~/Localization_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

The SLAM launch defaults to saving `~/maps/slam_map.pgm` and `~/maps/slam_map.yaml` every five seconds. Change the destination or disable saving with launch arguments:

```bash
ros2 launch robot_slam slam.launch.py output_dir:=/tmp/my_map map_name:=depot auto_save_interval:=10.0
ros2 launch robot_slam slam.launch.py auto_save:=false
```

Useful inspection commands:

```bash
ros2 topic echo /ekf/odom
ros2 topic echo /scan
ros2 topic echo /slam/pose
ros2 topic echo /map --qos-durability transient_local
ros2 run tf2_tools view_frames
```

The map saver writes a standard PGM image and YAML metadata. Stop the launch with `Ctrl+C` after the desired map has been saved.

## Running localization and A*

This launch uses the checked-in `map_publisher/maps/my_map.yaml` by default and starts Gazebo, the map publisher, EKF, particle filter, A* planner, motor command node, RViz, and bridge:

```bash
source ~/Localization_ws/install/setup.bash
ros2 launch map_publisher localization.launch.py
```

In RViz, use **2D Pose Estimate** to publish an initial pose on `/initialpose`. Then drive with the keyboard as above. Monitor the estimate and planner:

```bash
ros2 topic echo /amcl_pose
ros2 topic echo /particlecloud
ros2 topic echo /plan
```

Request a path. The planner intentionally uses the latest `/amcl_pose` as its start; the `start` field below is accepted by the ROS service for completeness but is ignored by the node:

```bash
ros2 service call /plan_path nav_msgs/srv/GetPlan \
"{start: {header: {frame_id: map}, pose: {orientation: {w: 1.0}}}, goal: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 1.0}, orientation: {w: 1.0}}}, tolerance: 0.0}"
```

To use a different saved map, pass its YAML path. Relative image paths are resolved beside that YAML file:

```bash
ros2 launch map_publisher localization.launch.py map_yaml:=/absolute/path/to/slam_map.yaml
```

For a map/RViz-only view (without Gazebo or localization), use:

```bash
ros2 launch map_publisher display.launch.py map_yaml:=/absolute/path/to/my_map.yaml
```

## Data flow and frame tree

```mermaid
flowchart LR
  G[Gazebo + bridge] --> O[/wheel_encoder/odom + IMU + /scan]
  O --> E[Wheel/IMU EKF]
  E --> P[Particle filter]
  P --> A[/amcl_pose + map→odom]
  M[Saved map] --> L[Map publisher]
  L --> P
  A --> S[A* planner]
```

The intended TF chain is `map → odom → base_link → rplidar_c1`. SLAM and the particle filter both publish a `map → odom` correction; run one localization source at a time to avoid competing TF broadcasters.

## Concepts and formulas

### Differential-drive motion and odometry

For wheel radius (r), wheel separation (b), and right/left angular wheel speeds (omega_R,omega_L):

\[
v=\frac{r}{2}(\omega_R+\omega_L),\qquad
\omega=\frac{r}{b}(\omega_R-\omega_L).
\]

The unicycle state is (x=[x,y,\theta]^T). Over a short interval (\Delta t), the implementation predicts

\[
x_{t+1}=x_t+v\Delta t\cos\theta_t,\quad
y_{t+1}=y_t+v\Delta t\sin\theta_t,\quad
\theta_{t+1}=\mathrm{wrap}(\theta_t+\omega\Delta t).
\]

The SLAM motion model also expresses two odometry readings as

\[
\delta_{rot1}=\operatorname{atan2}(y'-y,x'-x)-\theta,\quad
\delta_{trans}=\sqrt{(x'-x)^2+(y'-y)^2},\quad
\delta_{rot2}=\theta'-\theta-\delta_{rot1}.
\]

### EKF (wheel odometry + IMU yaw)

The Python EKF state is (x=[x,y,\theta]^T). Its prediction is the motion model above. Linearizing it gives

\[
F=\frac{\partial f}{\partial x}=\begin{bmatrix}
1&0&-v\Delta t\sin\theta\\
0&1& v\Delta t\cos\theta\\
0&0&1
\end{bmatrix},\qquad
G=\frac{\partial f}{\partial [v,\omega]}=\begin{bmatrix}
\Delta t\cos\theta&0\\
\Delta t\sin\theta&0\\
0&\Delta t
\end{bmatrix}.
\]

With control-noise covariance (Q_u=\operatorname{diag}(\sigma_v^2,\sigma_\omega^2)),

\[
P^- = F P F^T + GQ_uG^T.
\]

The IMU supplies a yaw measurement (z=\theta+v_z), so (H=[0\;0\;1]). The update is

\[
\nu=\operatorname{wrap}(z-Hx^-),\quad
S=HP^-H^T+R,\quad
K=P^-H^TS^{-1},
\]
\[
x^+=x^-+K\nu,\qquad P^+=(I-KH)P^-.
\]

Angles are wrapped to ((-\pi,\pi]). This EKF fuses only wheel odometry and IMU orientation; LiDAR is used later by the particle filter or custom SLAM, not directly in this EKF.

### SLAM and occupancy-grid mapping

SLAM estimates both the robot trajectory and map:

\[
p(x_{0:t},m\mid z_{1:t},u_{1:t}).
\]

The Bayes filter structure is

\[
p(x_t,m\mid z_{1:t},u_{1:t})\propto p(z_t\mid x_t,m)
\int p(x_t\mid u_t,x_{t-1})p(x_{t-1},m\mid z_{1:t-1},u_{1:t-1})dx_{t-1}.
\]

This workspace uses a lightweight deterministic approximation: odometry predicts pose, scan matching adjusts it, and each LiDAR ray updates a 2D grid. A cell is stored in log-odds form

\[
l_t(m_i)=l_{t-1}(m_i)+
\log\frac{p(m_i\mid z_t,x_t)}{1-p(m_i\mid z_t,x_t)}-
\log\frac{p_0}{1-p_0},
\]

with (p(m_i)=1/(1+e^{-l(m_i)})). Bresenham ray tracing marks cells before a hit as free and the endpoint as occupied; log-odds are clamped to prevent numerical saturation. The map is published as `nav_msgs/OccupancyGrid` (`0` free, `100` occupied, `-1` unknown).

### Scan matching

For a LiDAR point ((x_r,y_r)) and pose ((x,y,\theta)), the map-frame endpoint is

\[
\begin{bmatrix}x_w\\y_w\end{bmatrix}=
\begin{bmatrix}x\\y\end{bmatrix}+
\begin{bmatrix}\cos\theta&-\sin\theta\\\sin\theta&\cos\theta\end{bmatrix}
\begin{bmatrix}x_r\\y_r\end{bmatrix}.
\]

The matcher scores how likely transformed endpoints are occupied (bilinear interpolation of nearby grid cells) and performs finite-difference gradient ascent:

\[
\frac{\partial S}{\partial x}\approx\frac{S(x+\epsilon)-S(x-\epsilon)}{2\epsilon},
\]

with equivalent terms for (y) and (\theta). The pose update is constrained by a search window and a maximum iteration count. There is no loop closure or pose-graph optimization, so long trajectories accumulate drift.

### Particle-filter localization

Given a fixed map (m), localization follows

\[
bel(x_t)=\eta\,p(z_t\mid x_t,m)\int p(x_t\mid u_t,x_{t-1})bel(x_{t-1})dx_{t-1}.
\]

The node approximates this with particles. Each particle samples the odometry motion model, receives a LiDAR likelihood, and is resampled when the effective sample size is low:

\[
w_i\propto p(z_t\mid x_t^{(i)},m),\quad
\bar w_i=\frac{w_i}{\sum_jw_j},\quad
N_{eff}=\frac{1}{\sum_i\bar w_i^2}.
\]

The endpoint sensor model used here is

\[
p(z\mid x,m)=z_{hit}\exp\left(-\frac{d^2}{2\sigma^2}\right)+\frac{z_{rand}}{z_{max}},
\]

where (d) is the nearest occupied-map distance for a beam endpoint. The reported pose averages (x,y) directly and averages heading on the unit circle:

\[
\hat\theta=\operatorname{atan2}\left(\sum_i\bar w_i\sin\theta_i,\sum_i\bar w_i\cos\theta_i\right).
\]

### A* global planner

The planner inflates occupied cells by the robot footprint radius

\[
r_{footprint}=s\sqrt{(L/2)^2+(W/2)^2},
\]

where (s) is the safety margin. It then searches an 8-connected grid with cardinal cost (1), diagonal cost \(\sqrt2\), and Euclidean heuristic:

\[
f(n)=g(n)+h(n),\qquad h(n)=\sqrt{(x_n-x_g)^2+(y_n-y_g)^2}.
\]

The heuristic is admissible for these costs, so A* returns a least-cost grid path when one exists. World/grid conversion is

\[
g_x=\left\lfloor\frac{x-x_{origin}}{resolution}\right\rfloor,\qquad
g_y=\left\lfloor\frac{y-y_{origin}}{resolution}\right\rfloor.
\]

Unknown and inflated cells are treated as blocked. The node publishes `nav_msgs/Path`; it does not smooth the path, account for robot orientation/kinematics, or follow the path. A separate controller is needed to turn `/plan` into velocity commands.

## Important limitations and operating rules

- Run either `robot_slam slam.launch.py` **or** `map_publisher localization.launch.py` when testing TF; both can publish `map → odom`.
- The custom SLAM has no loop closure, scan submap, or pose-graph back end.
- The particle filter is an educational AMCL-style implementation, not Nav2 AMCL. Tune particle count, noise, and likelihood parameters for a new robot/map.
- The A* service uses the latest `/amcl_pose` as the start pose and requires a valid localized pose before planning.
- The map publisher resolves a relative `image:` entry relative to the YAML file, so saved maps can be moved as a pair.
- Verify sensor frame names in RViz/TF when changing the robot model; the expected LiDAR frame is `rplidar_c1`.

## Recommended project name

For consistency with the companion state-estimation workspaces, rename the repository to:

**`mobile_robot_mapping_localization`**

Suggested GitHub title: **Mobile Robot Mapping, Localization, and A* Navigation**

Suggested short description: *ROS 2 differential-drive simulation with custom 2D SLAM, wheel/IMU EKF, particle-filter localization, and an obstacle-aware A* global planner.*

## Demonstration videos

- [Map loading](https://drive.google.com/file/d/1qkuTcxFNs7JTO7KdOzCg5hBh9JWDEUM_/view?usp=sharing)
- [Localization](https://drive.google.com/file/d/1kVO9Y25D0f7bDvTiQgXmhmGv1QSmU44g/view?usp=sharing)
- [A* planner](https://drive.google.com/file/d/14wZ2aATygsQfqbmlcnIKxjOF1vi4FB0L/view?usp=sharing)
- [SLAM](https://drive.google.com/file/d/1BJv9KfPFIFWDRuL5zWYYKDz3L6aAskUS/view?usp=sharing)
