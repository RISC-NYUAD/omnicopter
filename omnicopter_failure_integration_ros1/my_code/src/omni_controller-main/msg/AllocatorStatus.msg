std_msgs/Header header
bool accepted
bool fallback
int32 solver_status
float64 solve_seconds
float64 candidate_constraint_violation
float64 issued_wrench_error
float64 step_radius
float64[2] tangent_step
float64[8] thrust

# Additional diagnostics; allocator command policy is unchanged.
float64 motor_bound_violation          # N, maximum positive candidate bound violation
float64 direction_violation            # maximum positive (norm(v)^2 - radius^2)
float64 candidate_step_norm
float64 solver_seconds                # elapsed at acceptance check, before fallback
bool rejected_by_status
bool rejected_by_constraints
bool rejected_by_time
# World-frame force vectors in N; allocated force excludes actuator lag.
float64[3] desired_force_world
float64[3] allocated_force_world

# Governed-recovery diagnostics. Original fields remain available.
string allocation_mode
bool feasible_attitude_found
bool allocation_converged
bool command_reused
float64[4] feasible_attitude_xyzw
float64 governor_seconds
float64 static_force_residual
float64 static_moment_residual
float64 force_tracking_error
float64 moment_tracking_error

# Manuscript mode: raw projected target is feasible_attitude_xyzw;
# transition reference is rate/acceleration limited and need not be statically feasible.
float64[4] transition_reference_xyzw
# xi = A*u - w_hat(v); N then Nm, body frame. Not measured plant residual.
float64[6] wrench_slack
float64[3] local_direction_body
float64[3] reference_rate_world
float64 joint_objective
