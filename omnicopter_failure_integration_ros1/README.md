# Omnicopter: actuator-constrained trajectory controller

This is a first **simulation integration**, with a compiled and numerically tested C++ allocator. The ROS node, launch integration and Gazebo modifications require a build and end-to-end run in your ROS1 Noetic / Gazebo 11 environment. This environment did not contain ROS or Gazebo, so no Gazebo flight validation is claimed.

The original controller source and `simulate.launch` are retained. Use the new `failure_aware.launch` for the new controller; run only one simulation/controller mode at a time. Shared simulator changes include input validation and a corrected transformation of rotor drag torque into the parent-link frame, so the legacy simulation is not byte-for-byte behaviorally identical.

## Build and run

Use `my_code/` as a separate catkin workspace, or copy the modified packages into a separate copy of your existing working workspace. Do not put duplicate copies of these packages on the same ROS package path.

```bash
sudo apt install libeigen3-dev libnlopt-dev python3-numpy python3-yaml
source /opt/ros/noetic/setup.bash
cd /path/to/omnicopter_failure_integration/my_code
catkin_make
source devel/setup.bash
roslaunch omni_firmware failure_aware.launch
```

The simulation launches paused, as in the supplied simulator. In another terminal, source this workspace and use the existing maneuver services:

```bash
rosservice call /gazebo/unpause_physics
rosservice call /ArmDisarm
rosservice call /lift_off "height: 1.3
duration: 7.0"
```

Wait for the takeoff trajectory to finish. Start with gentle motions, for example:

```bash
rosservice call /go_to_6D "{x: 0.3, y: 0.0, z: 1.3, roll: 0.0, pitch: 0.2, yaw: 0.0, duration: 6.0}"
```

Full flips are outside the local attitude model's intended validation scope. The existing `LinearWrench` / `AngularWrench` test services are provided by the legacy controller, not the new node.

## Inject a known actuator fault

Motor order is **zero based**: `0` means SDF `rotor_0`, corresponding to the old interface's `propeller1`. `7` means `rotor_7` / `propeller8`.

```bash
# Keep motor 5 at no more than one third of its nominal maximum THRUST.
rosrun controller set_motor_limit.py --motor 5 --fraction 0.3333333333

# Restore every motor's nominal limits.
rosrun controller set_motor_limit.py --restore

# Complete shutdown of motor 6.
rosrun controller set_motor_limit.py --motor 6 --fraction 0
```

The fraction applies to thrust, not RPM. Commands update `/failure_controller/set_motor_limits`, and the node publishes the resulting bounds on latched `/omnicopter/motor_limits`. Both the optimizer and the Gazebo motor plugin receive these limits. The plugin caps physical thrust even while its motor-speed state is still decaying, so a shutdown immediately produces zero thrust. This represents a known thrust-cap fault; it does not model a windmilling propeller, a stuck motor, a changed thrust coefficient, or sensor-based fault detection. The physical lower force is zero when stopped; configured nonzero lower bounds constrain active motor commands, not a disarmed vehicle.

Some failures leave the current force/attitude request infeasible. An accepted service response means the bounds were applied, not that the vehicle can still track its trajectory.

## Method implemented

The implementation follows the supplied paper's local direction optimization, extended to the eight fixed-axis motors using the supplied handoff's general tangent-basis formulation. It is not an exact reproduction of the paper's dual-tilt platform or a numerical reproduction of the unavailable Simulink model.

The physical convention is

```
[force_body (N); moment_body (Nm)] = A * u
u[i] = rotor thrust (N)
R maps body coordinates to the world frame; world gravity is [0, 0, -9.81].
Measured angular velocity is expressed in body coordinates.
```

For nominal rank-six `A`, precompute `P = pseudoinverse(A)` and an orthonormal `B` spanning its null space. `P_f` and `P_m` are the force and moment blocks of `P`. A complete failure is represented by a zero motor bound while retaining nominal `A`; rank-six formulas are never applied to an unverified reduced matrix.

At each step:

```
f_world = mass*(a_ref + 9.81*e3) + Kp*(p_ref-p) + Kv*(v_ref-v) + Ki*integral
F       = norm(f_world)
d_B     = R.transpose()*f_world/F
d_ref   = R_ref.transpose()*f_world/F
C       = orthonormal tangent basis at d_B
v_ref   = C.transpose()*d_ref
r       = min(norm(v_ref), max_v)
H       = diag(1,1,0)
m_base  = omega cross (J*omega) - K_omega*omega - K_R*e3*e_z
M       = -P_m*K_R*H*skew(d_B)*C
u(v,x)  = P_f*F*(C*v + d_B*sqrt(1-v.transpose()*v)) + P_m*m_base + M*v + B*x
```

Optimize four variables `[v(2); x(2)]`:

```
min  alpha/2*||v-v_ref||² + beta/2*||x||² + gamma/2*||u-u_previous||²
s.t. lower[i] <= u[i] <= upper[i]    for i=0,...,7
     ||v||² <= r²
```

NLopt SLSQP uses analytic objective and constraint gradients. This is a nonlinear constrained solve; the existing QuadProg++ allocation cannot directly solve it. No global-optimality claim is made. Component bounds keep all trial evaluations in the square root's valid domain. When `r` is effectively zero, `v` is fixed to zero and only null-space allocation is solved. The tangent basis is reconstructed robustly each step and its coordinates are reset rather than incorrectly reusing a previous basis.

The optimizer output is already the allocated motor thrust. It is not passed through the old wrench mixer. `d(v)` is a prediction in the local control model, not an instantaneous physical rotation.

## Defaults and deliberate differences

Edit `my_code/src/omni_controller-main/config/failure_aware.yaml` **before launch**. Parameters are read at startup; changing them with `rosparam set` during a run does not reconfigure the node. Motor limits have a live service.

| Item | Implemented choice |
| --- | --- |
| Objective reference | Raw tangent reference and bounded step, following the paper's Eq. (19) |
| MATLAB reference variant | Set `clip_reference: true` to clip the objective reference like the supplied MATLAB |
| Null-space penalty | `beta: 1e-5`; necessary for the eight-motor extension |
| Smoothing | Implemented; `gamma: 0.0` by default, matching the supplied MATLAB omission. Try a small positive value such as `1e-4` to enable the paper's term, then evaluate tracking and timing. |
| Max local step | `max_v: 0.1`; this is a dimensionless direction increment, not a motor slew-rate bound |
| Angular feedback | Body-rate damping and yaw reference error; roll/pitch correction comes from the optimization |
| Angular feedforward | Omitted, as in the paper's simplified optimization and supplied reference |
| Gyroscopic compensation | Standard `+omega cross (J*omega)`; deliberately corrects the source MATLAB sign discrepancy |
| Position integral | Your supplied gains `[6, 6, 8]` are retained; accumulation freezes during fallback |
| Inertia and allocation | Generated from the supplied SDF instead of the old controller's rounded matrix and uninitialized inertia buffer |
| Maximum motor thrust | 38.6336551827 N, inferred from the old 1900-us output ceiling, old adapter's PWM-to-RPM fit, and SDF thrust coefficient. This is a simulation default, not measured hardware capability. |
| Command interface | `/omnicopter/motor_thrust` in N; adapter analytically inverts the plugin's thrust law into its normalized RPM interface |
| Timing | 250 Hz controller, 2-ms cooperative optimizer limit, 80 evaluations; the time limit is not a hard real-time guarantee |

The aggregate rigid-body mass is 3.44 kg, including the SDF's base, IMU and rotor links. Inertia includes their positions and orientations. The new adapter converts the base-link CoG measurement to the aggregate CoM using a generated offset. The generator assumes the supplied rigid geometry and rotor arrangement; it does not identify real hardware parameters or reproduce every effect of articulated Gazebo rotor joints.

If you change the SDF geometry, masses, inertias, thrust coefficient or rotor spin signs, regenerate the model configuration and test fixture:

```bash
python3 src/omni_controller-main/tools/model_config.py
```

This overwrites both generated YAML files and the C++ model fixture, including any hand-edited gains in `failure_aware.yaml`. Reapply your tuning afterward.

## Acceptance and fallback

Solver convergence alone is insufficient. The candidate must be finite and independently satisfy every motor bound and the local-step constraint within tolerance. Timeout, evaluation exhaustion, failed solver status or a failed feasibility check enters fallback. Tiny accepted bound roundoff is cleaned up; infeasible nonlinear iterates are not repaired by clipping.

The fallback solves a bounded least-squares approximation to the original geometric-controller wrench, with force residual weighted more heavily than moment residual. It is a numerical best-effort policy added for this integration, not a strategy or stability guarantee provided by the paper. It can sacrifice force and attitude tracking. It does not establish a controlled landing, recovery or safe operation for an arbitrary fault. Total actuator loss produces zero commands.

The adapter sends a zero target if controller messages stop for 0.1 s. The node sends zero commands when disarmed, missing a valid reference/state, or receiving stale state. The Gazebo plugin retains the supplied actuator-speed smoothing dynamics.

A critical local-model limitation remains: when the tangent reference is zero, the paper's step radius is zero. The solver cannot invent an unrelated attitude maneuver to escape an infeasible hover. Large and antipodal attitude differences also require a separate global attitude strategy. Eight motors and nominal rank six do not imply that every single-motor failure leaves every pose feasible.

## Observe and record

```bash
rostopic echo /failure_controller/status
rosbag record /pose_full /pose_d /failure_controller/status /omnicopter/motor_thrust /omnicopter/motor_limits
```

`accepted` means the nonlinear candidate passed the convergence/resource and feasibility checks. `fallback` means the issued thrust comes from the bounded approximation. `candidate_constraint_violation` describes the nonlinear candidate, not the issued fallback. `issued_wrench_error` is an unscaled diagnostic norm against the original unconstrained force/full-attitude wrench; it combines numerical N and Nm components and is not a unit-consistent performance score. Prefer position, attitude and individual force/moment plots for evaluation. `solve_seconds` measures the entire allocator step, including fallback when used.

## Local validation

`validation/allocator_checks.txt` contains the successful C++ regression-check output. Checks include:

- `A*P=I`, `A*B=0`, and orthonormal null and tangent bases;
- analytic objective and constraint Jacobians versus finite differences;
- finite trial evaluations and the zero-radius special case;
- nominal hover where the pseudoinverse alone produces negative commands, requiring null-space redistribution;
- nonzero attitude steps, all eight single-motor shutdown requests, and issued command bounds;
- impossible/all-zero motor bounds, zero desired force, bad bounds and rank;
- clipped-reference mode and time/evaluation-limit fallback.

In the particular nonzero-pitch single-fault test, **2 of 8** requests were feasible and accepted; the others used bounded fallback. Passing the bounds tests is not a claim of trajectory preservation for all eight faults.

`validation/offline_sim.csv` is a 20-s / 5000-step rigid-body run of the same C++ allocator. It includes a smooth 0.4-m translation with 0.4-rad pitch, motor 5 capped at one third thrust from 6-10 s, restoration, and motor 6 stopped from 14 s. Motor-speed smoothing is included. This initializes in airborne hover and does not test takeoff, landing, contacts, ROS timing or Gazebo dynamics. See `validation/summary.md` for measured results and their limits.

To build and rerun the numerical checks without ROS, from `my_code/`:

```bash
cmake -S src/omni_controller-main/test -B build_allocator -DCMAKE_BUILD_TYPE=Release
cmake --build build_allocator -j2
ctest --test-dir build_allocator --output-on-failure
./build_allocator/offline_sim offline_sim.csv
```

Local execution used Eigen 3.4.0 and NLopt 2.11.0. ROS/Gazebo code and catkin builds were checked by source inspection and XML/Python syntax checks only. MATLAB/Simulink numerical parity was not tested because the supplied material contains no captured reference input/output fixtures or runnable Simulink model.

## Main files

| File under `my_code/src/` | Purpose |
| --- | --- |
| `omni_controller-main/include/feasible_allocator.hpp` | ROS-independent nonlinear allocator and bounded fallback |
| `omni_controller-main/src/failure_aware_node.cpp` | ROS feedback loop, timing, arming and fault-limit service |
| `omni_controller-main/config/failure_aware.yaml` | Physical model, gains, limits and optimization settings |
| `omni_controller-main/config/failure_sim.yaml` | Direct-thrust adapter configuration |
| `omni_controller-main/srv/SetMotorLimits.srv` | Runtime known-fault injection interface |
| `omni_controller-main/msg/AllocatorStatus.msg` | Solver and issued-command diagnostics |
| `omni_controller-main/tools/set_motor_limit.py` | Convenient partial/complete-fault commands |
| `omni_controller-main/tools/model_config.py` | Reproducible SDF-to-model conversion |
| `omni_firmware-main/launch/failure_aware.launch` | New simulation entry point |
| `AeRoSim/uav_simulator/nodes/adapter.cpp` | Newtons-to-RPM path, timestamp and CoM conversion |
| `AeRoSim/uav_simulator/src/beta_motor_plugin.cpp` | Physical fault caps, input checks and drag-frame correction |

The supplied `paper_data/` is included unchanged for comparison. No hardware command interface is enabled by this integration.
