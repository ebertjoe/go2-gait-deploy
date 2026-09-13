# Sim-2-Real Transfer — Unitree Go2

ROS 2 workspace for running a learned locomotion policy on a Unitree Go2, in
MuJoCo simulation and on the real robot. The control chain is **identical** in
both cases — only the last hop changes:

```
                +------------------+                    +--------------------+
  /joy  ------> |  low_level_ctrl  | --- /mujoco/lowcmd -> | mujoco_simulator |  (sim)
                |  (state machine) |     or /lowcmd     -> |  Go2 firmware    |  (real)
                +------------------+                    +--------------------+
                     ^        |                                    |
       /rl/target_pos|        +---------- lowstate ----------------+
                     |                                             |
                +----+-------------+                               |
                |    rl_policy     | <-----------------------------+
                | (53-dim obs,     |
                |  Raibert gait)   |
                +------------------+
```

Because the simulator only plays the part of the hardware — physics and
sensors, no policy logic — the code path you validate in MuJoCo is the one that
gets deployed.

This README covers **getting the simulator + policy running on a fresh
machine**. For the upstream framework description (control logic, EKF base
velocity estimator, real-robot deployment notes) see
[`src/Deploy-an-RL-policy-on-the-Unitree-Go2-robot/README.md`](src/Deploy-an-RL-policy-on-the-Unitree-Go2-robot/README.md).

---

## 1. Prerequisites

| What | Version used here | Notes |
|---|---|---|
| Ubuntu | 22.04 | 20.04 + Foxy also works, see the upstream README's Python 3.8 caveat |
| ROS 2 | Humble | `sudo apt install ros-humble-desktop ros-humble-joy` |
| Python | 3.10 | Humble's system Python — no venv needed, and a venv tends to hide `rclpy` |
| MuJoCo | 3.2.3 | `pip install mujoco==3.2.3` |
| PyTorch | 2.x (CPU is fine) | policies are TorchScript, inference runs single-threaded on CPU |
| NumPy | 1.26.x | **not** 2.x — MuJoCo 3.2.3 and the ROS 2 Humble Python stack expect 1.x |
| `unitree_ros2` | — | provides the `unitree_go` messages (`LowCmd` / `LowState`) |

```bash
pip install "numpy<2" "mujoco==3.2.3" torch pyyaml
```

A **display is required** for the simulator: `mujoco_simulator.py` opens a
passive MuJoCo viewer and will not start headless.

A **gamepad** (Xbox-style, on `/dev/input/js0`) is the intended way to drive the
state machine. §5 shows how to fake it from the command line if you don't have
one.

### unitree_ros2

The `unitree_go` message package comes from Unitree and is not vendored here:

```bash
git clone https://github.com/unitreerobotics/unitree_ros2 ~/unitree_ros2
cd ~/unitree_ros2/cyclonedds_ws/src
git clone https://github.com/ros2/rmw_cyclonedds -b humble
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x
cd ~/unitree_ros2/cyclonedds_ws
colcon build --packages-select cyclonedds
source /opt/ros/humble/setup.bash
colcon build
```

It ships two environment scripts. **For simulation, always use the local one:**

- `setup_local.sh` — binds CycloneDDS to the loopback interface `lo`. Use this
  for MuJoCo.
- `setup.sh` — binds to a physical NIC (`enp3s0` in the stock file, edit to
  match your machine). Use this only when talking to the real robot.

Sourcing `setup.sh` for a sim run is the most common reason nodes come up but
never see each other's topics.

---

## 2. Clone

```bash
git clone https://github.com/ebertjoe/go2-gait-deploy.git ros2_ws
cd ros2_ws

# Sanity check: this must print the path, not an error.
ls resources/go2/scene_flat.xml
```

The workspace root carries a symlink
`resources -> src/Deploy-an-RL-policy-on-the-Unitree-Go2-robot/resources`, and
both nodes locate the robot model and the policy checkpoints through it. It is
committed as a **relative** link, so it works from any clone location. If that
`ls` fails, recreate it:

```bash
ln -sfn src/Deploy-an-RL-policy-on-the-Unitree-Go2-robot/resources resources
```

---

## 3. Build

```bash
source ~/unitree_ros2/setup_local.sh     # ROS 2 + unitree_go messages + CycloneDDS
cd ~/ros2_ws
colcon build --symlink-install --packages-select deploy_rl_policy
```

`deploy_rl_policy` is all you need for simulation. The workspace also contains
`base_velocity_estimator` (EKF base-velocity estimation), which additionally
requires [Pinocchio](https://github.com/stack-of-tasks/pinocchio) 3.x and
Eigen3. The current 53-dim policy does **not** take base velocity in its
observation, so skip it unless you need it:

```bash
colcon build --symlink-install          # builds both, needs Pinocchio
```

Rebuild after editing any script — the Python nodes are installed into
`install/deploy_rl_policy/lib/deploy_rl_policy/`, and without
`--symlink-install` they are *copies*, so source edits have no effect until you
build again.

---

## 4. Run the simulator and the policy

Four terminals. **Source both setup scripts in every one of them**, in this
order:

```bash
source ~/unitree_ros2/setup_local.sh
source ~/ros2_ws/install/setup.bash
```

**Terminal 1 — MuJoCo physics + sensor server**

```bash
ros2 run deploy_rl_policy mujoco_simulator.py
```

The viewer opens and the robot hangs in its reset pose. It deliberately does
not step physics until the first `/mujoco/lowcmd` arrives — otherwise it would
just collapse on the floor.

**Terminal 2 — low-level state machine**

```bash
ros2 run deploy_rl_policy low_level_ctrl --ros-args -p is_simulation:=true
```

The robot drops into the lay-down pose. This node owns the PD loop and is the
only thing that ever writes `lowcmd`.

**Terminal 3 — RL policy**

```bash
ros2 run deploy_rl_policy rl_policy.py --ros-args -p is_simulation:=true
```

Loads the TorchScript checkpoint, warms up the JIT, and waits. It logs
`rl_policy ready — SIM  lowstate<-/mujoco/lowstate  targets->/rl/target_pos`.
It publishes nothing until you engage it.

**Terminal 4 — gamepad**

```bash
ros2 run joy joy_node
```

---

## 5. Driving it

The robot is a three-state machine: **lay down → stand up → run policy**.

| Input | Effect |
|---|---|
| *(startup)* | robot settles into the lay-down pose |
| **B** | lay down → stand up |
| **A** | stand up → lay down |
| **LB + RB** (held, while standing) | run the RL policy |
| release LB/RB | policy disengages, robot holds the standing pose |
| LT + RT together | shuts `low_level_ctrl` down |

Press **B**, wait for the stand-up to finish, then hold **LB + RB**.

`rl_policy` refuses to engage unless the robot is actually upright and settled —
tilt < 0.15, `|ang_vel|` < 0.3 rad/s, joint error < 0.30 rad from the standing
pose, and fresh `lowstate`. If it logs `Activation REJECTED: pose_err=…`, the
stand-up simply hasn't finished; wait a second and try again.

### No gamepad?

Publish `/joy` by hand. Keep each command running (it repeats at 1 Hz, which is
exactly like holding the button) and Ctrl-C it before moving to the next.

```bash
# Stand up (button B = index 1). Leave running ~3 s, then Ctrl-C.
ros2 topic pub /joy sensor_msgs/msg/Joy \
  "{axes: [0,0,0,0,0,0,0,0], buttons: [0,1,0,0,0,0,0,0,0,0,0]}"

# Engage the policy (LB = index 4, RB = index 5). Leave this one running.
ros2 topic pub /joy sensor_msgs/msg/Joy \
  "{axes: [0,0,0,0,0,0,0,0], buttons: [0,0,0,0,1,1,0,0,0,0,0]}"
```

Send at least 8 axes and 11 buttons — `low_level_ctrl` indexes `axes[5]` and
`buttons[5]` without a bounds check. Keep the axes at `0`: `axes[2] == -1 &&
axes[5] == -1` is the trigger-combo shutdown.

### What you should see

Terminal 1 prints `height=… contact=[…]` every second — base height should sit
around **0.30–0.33 m** while walking. Terminal 3 prints the observation vector
once at activation, then a status line every 50 steps.

In simulation the velocity command and gait follow a fixed schedule baked into
`rl_policy.py` (`GAIT_SCHEDULE_SIM`): 5 s standing, then 20 s of bound at
0.8 m/s, then one gait every 5 s — trot, hop, amble, pronk, limp, run — ending
in a sustained run. The sticks are not wired to the velocity command.

---

## 6. Using a different policy or gait schedule

Checkpoints live in `resources/go2/*.pt` (TorchScript exports). One policy is
included, `resources/go2/policyAfterChapter6.pt`, and it is the default `POLICY_PATH`
in `rl_policy.py`, so the sim runs out of the box. To use your own, place the
`.pt` file in `resources/go2/` and pass `policy_path` as shown below:

```bash
ros2 run deploy_rl_policy rl_policy.py --ros-args \
  -p is_simulation:=true \
  -p policy_path:=$HOME/ros2_ws/resources/go2/my_policy.pt
```

Any replacement must match the interface hard-coded in `rl_policy.py`:

- **53-dim observation**, in this order — `projected_gravity` (3),
  `joint_pos` (12), `ang_vel` (3), `joint_vel` (12), `vel_cmd` (3),
  `foot_contact` (4), `desFeetContact` (4), `refFootZ` (4), `refFootX` (4),
  `refFootY` (4)
- **12 actions**, in Isaac-internal joint order (grouped by joint type,
  legs FL/FR/RL/RR), scaled by `ACTION_SCALE = 0.25` and added to the default
  pose. `INTERNAL_TO_MUJOCO` reorders them to SDK order before publishing.
- **100 Hz** (`STEP_DT = 0.010`)

Other knobs, all near the top of
`src/Deploy-an-RL-policy-on-the-Unitree-Go2-robot/src/deploy_rl_policy/scripts/rl_policy.py`:

| Knob | Meaning |
|---|---|
| `GAIT_TABLE` | period, duty threshold, leg offsets, nominal foot height per gait (0=bound, 1=trot, 2=hop, 3=amble, 4=pronk, 5=limp, 6=stand, 7=run) |
| `GAIT_SCHEDULE_SIM` | `(duration_steps, gait_id, [vx, vy, wz])` sequence used in sim |
| `GAIT_SCHEDULE_REAL` | conservative schedule used when `is_simulation:=false` |
| `FOOT_FORCE_THRESHOLD` | contact binarisation; 20.0 works in sim (the simulator reports 0/100), **recalibrate on hardware** |
| `JOINT_LIMIT_LO/HI` | safety clamp on published targets |

Policy-phase PD gains are **not** ROS parameters — they are hard-coded as
`kp = 30`, `kd = 0.75` in `run_policy()` in `low_level_ctrl.cpp`. (The docstring
at the top of `rl_policy.py` mentions `policy_kp` / `policy_kd` parameters;
those don't exist in the current `low_level_ctrl`.)

The terrain is chosen by `xml_path` in `mujoco_simulator.py` — swap
`scene_flat.xml` for `scene_terrain.xml` for the height-field scene.

`src/deploy_rl_policy/configs/go2.yaml` and `scripts/config.py` are leftovers
from an earlier 270-dim observation setup and are not read by the current
nodes — ignore them.

---

## 7. Troubleshooting

| Symptom | Cause |
|---|---|
| `FileNotFoundError: .../resources/go2/scene_flat.xml` | the `resources` symlink is dangling — see §2 |
| `ModuleNotFoundError: No module named 'unitree_go'` | `unitree_ros2` not sourced in this terminal |
| Nodes start, but nothing moves; `ros2 topic echo /mujoco/lowstate` is silent | mismatched DDS config — source `setup_local.sh` (not `setup.sh`) in *every* terminal, and check `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` is the same everywhere |
| Robot hangs mid-air, viewer open, nothing happens | `low_level_ctrl` isn't running — the simulator holds the reset pose until the first `lowcmd` |
| `Activation REJECTED: pose_err=…` | stand-up not finished, or the robot isn't upright. Press B, wait, retry |
| `Activation REJECTED: lowstate is stale` | the simulator died, or DDS isn't carrying `/mujoco/lowstate` |
| `Policy tick overran: … ms` occasionally | CPU contention. Force single-threaded BLAS (`OMP_NUM_THREADS=1`) if it's persistent |
| `lowstate STALE — deactivating` | the simulator stopped publishing; the policy disengages by design |
| Numpy/MuJoCo import errors after a `pip install -U` | NumPy 2.x got pulled in. `pip install "numpy<2"` |
| Edits to a script have no effect | rebuild — installed Python files are copies unless you used `--symlink-install` |

---

## 8. Real robot

Same three commands with `is_simulation:=false`, no `mujoco_simulator.py`,
source `setup.sh` (with the `NetworkInterface name` edited to your NIC) instead
of `setup_local.sh`. `rl_policy` then switches to `GAIT_SCHEDULE_REAL`
(stand → trot in place → 0.3 m/s trot).

Before anything else: **turn off the Go2's sport mode service**, otherwise the
onboard controller fights yours for the joints. And recalibrate
`FOOT_FORCE_THRESHOLD` — hang the robot (expect ~0), stand it, read
`foot_force`, set to roughly half the standing value. The upstream README has
the details.

---

## 9. Repo layout

```
ros2_ws/
├── resources -> src/.../resources        # symlink, see §2
└── src/Deploy-an-RL-policy-on-the-Unitree-Go2-robot/
    ├── resources/go2/                    # go2.xml, scene_*.xml, meshes, policyAfterChapter6.pt
    ├── src/deploy_rl_policy/
    │   ├── src/low_level_ctrl.cpp        # state machine + PD, the only lowcmd writer
    │   └── scripts/
    │       ├── mujoco_simulator.py       # physics + sensors, stands in for hardware
    │       ├── rl_policy.py              # 53-dim obs, Raibert gait planner, 100 Hz
    │       └── xbox_command.py           # /joy helper
    └── src/base_velocity_estimator/      # EKF base velocity (Pinocchio); optional
```
