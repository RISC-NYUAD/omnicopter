# Numerical validation results

The C++ allocator was compiled with C++14, Eigen 3.4.0 and the runtime's NLopt 2.11.0 library. All checks in `allocator_checks.cpp` passed; see `allocator_checks.txt`.

A separate 20-s rigid-body integration used the same allocator at 250 Hz, initialized in airborne hover. The commanded trajectory moves 0.4 m along x and ramps pitch to 0.4 rad over 2-5 s. Position integral feedback and rotor-speed smoothing are included. These are local software measurements, not Gazebo, hardware or paper-reproduction results.

| Phase | Time (s) | Max position error (m) | End position error (m) | End attitude error (deg) | Fallback steps |
| --- | --- | --- | --- | --- | --- |
| Healthy motion | 0-6 | 0.190859 | 0.048478 | 1.951 | 0 |
| Motor 5 at one third thrust | 6-10 | 0.119095 | 0.048070 | 70.602 | 150 |
| All limits restored | 10-14 | 0.047921 | 0.006577 | 41.716 | 2 |
| Motor 6 stopped | 14-20 | 0.331830 | 0.023641 | 29.351 | 89 |

Accepted nonlinear commands: 4759/5000. Median allocator step: 0.167 ms; 99th percentile: 0.436 ms; largest observed: 2.227 ms. Wall-clock measurements depend on host load and are not deadline guarantees. `solve_ms` includes fallback when used.

Final sampled position error is 0.024 m, while final attitude error is 29.35 degrees. The largest transient position error was 0.332 m after a loaded motor was stopped. Position PID gains [15,15,18], [10,10,12], [6,6,8] are retained from the supplied controller, with bounded error integration that freezes during fallback. The optimizer compromises reference attitude under actuator constraints; these results do not establish global attitude optimality or guaranteed recovery.

The eight individual shutdown checks use one specific current pose and reference request. Only 2/8 were feasible under that local request; every issued command in all eight cases obeyed the corresponding bounds. The dynamic run caps loaded motor 5 at one third thrust at 6 s and stops loaded motor 6 at 14 s. Their commands immediately before the faults were approximately 20.48 N and 18.32 N respectively, so these are binding faults. No failure detection is assumed or tested. The run does not establish recovery from arbitrary motor failures.

No ROS node compilation, catkin build, live service test, takeoff/landing test, Gazebo run, hardware test, or MATLAB numerical-parity comparison was possible here. XML and Python files passed syntax checks. Numerical derivatives were checked against finite differences, not against captured MATLAB results.
