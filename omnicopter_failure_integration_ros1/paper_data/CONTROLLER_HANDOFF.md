# C++ controller handoff: MATLAB FA controller to an octorotor

This is a source-level implementation brief, not a tested flight controller. It distinguishes the existing MATLAB behavior from the proposed eight-motor extension. The receiving developer supplies the actual allocation matrix and platform conventions before implementing the extension.

## Sources and scope

- Model: `D:/KU Windows/p2022b/Software/simulation_full_QP_w_md_FA.slx`.
- Main MATLAB Function block: `Controller/Optimal Control and Path`, function `OptimalControlPathInput`; includes local functions `quadobj` and `norm_cons`.
- Other relevant blocks: `Controller/position controller` and `Controller/Attitude controller`.
- Initialization: `D:/KU Windows/p2022b/Software/init_FA.m`.
- Paper: *Full-Pose Trajectory Tracking of Overactuated Multi-Rotor Aerial Vehicles With Limited Actuation Abilities*, especially Eqs. (14), (16), (18)-(20), printed pp. 4954-4955. PDF supplied in the user's Downloads folder.

The existing FA implementation optimizes two tangent-plane coordinates. It retains nonlinear actuator constraints, omits the control-smoothing objective, and has no null-space allocation variable because its six-input allocation matrix is square and full rank. The paper itself already uses a local/small-angle attitude model.

## Required platform contract

Obtain these from the platform owner; do not infer them from the MATLAB constants:

1. Allocation matrix A, its dimensions, motor order, units, and equation. This brief assumes a constant A in R^(6x8) with [force_body; torque_body] = A*u, for eight independently commanded fixed-axis motors.
2. What u means: rotor thrust, squared speed, or another calibrated quantity. In init_FA, u_max = w_max^2 and A contains force/torque coefficients. u is not automatically normalized throttle or PWM.
3. Frame and sign conventions: ENU/NED, FLU/FRD, gravity vector, rotation direction, quaternion order, torque signs, angular velocity frame.
4. Individual lower/upper motor bounds, motor-command conversion, actuator dynamics, command interface, and any rate limits.
5. Mass, full inertia tensor, feedback gains, estimator timestamps, control period, available solve time, and the existing flight stack's fallback behavior.

Check numerical rank of A. Eight motors do NOT imply six independent wrench directions. This formulation assumes rank(A)=6. For a conventional coplanar octorotor, this can fail; an underactuated attitude/thrust controller or a separately derived constrained formulation is then required. Do not silently use the full-row-rank formulas below on a rank-deficient matrix.

## Existing control cycle

Notation: R maps body coordinates into world coordinates; R_ref is the reference attitude; omega is measured body angular velocity. S(a)*b = a cross b. C has two orthonormal columns tangent to d_B.

### 1. Translational feedback

The MATLAB position block uses reference-minus-measured errors:

    e_p = componentwise_clip(p_ref - p, +/-e_p_sat)
    e_v = componentwise_clip(v_ref - v, +/-e_v_sat)
    f_world = mass*a_ref + Kp*e_p + Kv*e_v + Ki*integral_error + mass*g*e3

The gravity compensation sign above is the source convention. Convert it for the actual platform. The initializer sets the position integral gain to zero and both error limits to 0.5 in their respective units. Do not copy gains/limits without platform tuning.

### 2. Baseline torque and allocation

The source attitude block computes:

    E = -0.5*(R_ref.transpose()*R - R.transpose()*R_ref)
    e_att = vee(E)
    m_base_source = K_R*[0,0,e_att.z] - K_omega*omega + S(J*omega)*omega
    u_moment = P_m*m_base_source

Only the z component of the attitude error is used here. The optimization supplies the remaining attitude correction; do not add a second independent full roll/pitch reference torque on top.

IMPORTANT source discrepancy: S(J*omega)*omega equals -(omega cross J*omega). The paper's Eq. (17) uses +(omega cross J*omega). The source initializer uses isotropic J, so this difference vanishes there. With the real inertia tensor, reconcile this against the platform dynamics. Under the standard equation J*omega_dot + omega cross J*omega = torque, the compensation is +omega cross J*omega. Keep a source-parity mode separate from a deliberately corrected implementation.

The source omits desired angular-velocity tracking and angular-acceleration feedforward, consistently with the paper's simplified optimization controller.

### 3. Force directions and tangent coordinates

    F = norm(f_world)
    d_B = R.transpose()*f_world / F
    d_ref = R_ref.transpose()*f_world / F

Guard F near zero before normalization.

Source basis construction:

    t = d_B cross e3
    if norm(t) < 1e-3:
        C = [e1,e2]
    else:
        C = [normalize(t), normalize(d_B cross t)]

The fallback is exactly tangent only when d_B is exactly parallel to e3. For production, construct an orthonormal tangent basis using an axis sufficiently nonparallel to d_B and verify C.transpose()*d_B approximately zero. This is an intentional numerical improvement, not literal parity.

    v_ref_raw = C.transpose()*(d_ref - dot(d_B,d_ref)*d_B)
    v_ref = norm_clip(v_ref_raw, max_v)

The original uses MATLAB left division C\projection, equivalent to the transpose for orthonormal C. max_v=0.1 in init_FA. It clips the reference in the objective as well as limiting the step; Eq. (19) in the paper bounds the decision variable without replacing the reference in the objective. Preserve or change this explicitly.

This is a local direction model. In particular, antipodal d_ref and d_B project to zero; it is not a global attitude planner. max_v is a local dimensionless direction bound, not a commanded angular velocity or an actuator slew limit.

### 4. Nonlinear allocation map

Let P_f and P_m be the force and torque blocks of A's pseudoinverse. For full-row-rank A:

    P = pseudoinverse(A)           // 8x6 for the new platform
    P_f = P.leftCols(3)
    P_m = P.rightCols(3)
    H = diag(1,1,0)
    M = -P_m*K_R*H*S(d_B)*C
    s(v) = sqrt(1 - v.transpose()*C.transpose()*C*v)
    d(v) = C*v + d_B*s(v)
    u(v) = P_f*F*d(v) + u_moment + M*v

The source obtains P_f and P_m by separate force/torque-null-space constructions; using blocks of the full pseudoinverse yields the same minimum-norm decoupled mappings under the full-row-rank assumption. Verify A*P_f=[I;0] and A*P_m=[0;I].

M above reproduces the actual source including H; do not replace it blindly with a fixed 3x2 selector when using a general tangent basis.

### 5. Actual FA optimization

    minimize over v in R^2:  0.5*||v-v_ref||^2
    subject to:             0 <= u_i(v) <= u_max, i=1,...,6
                            ||v||^2 <= ||v_ref||^2

The four source `weights` entries are read but unused. There is no previous-control penalty. The initial guess is zero each call. MATLAB uses fmincon, Algorithm='sqp', StepTolerance=1e-4, with analytic objective and constraint gradients. This is a nonlinear constrained solve, not a single standard QP. The filename does not specify the mathematical solver class.

After solving, the source directly returns u(v). The returned vector is already the allocated motor command, not a wrench to pass through another mixer. The physical force is R*A_force*u in the current attitude; the predicted direction d(v) is not an instantaneous change in the actual body orientation.

## Proposed extension for a full-rank eight-motor platform

Changing six bounds to eight alone ignores the new platform's redundancy. Compute an orthonormal null basis B of A using a rank-revealing factorization. For rank(A)=6, B is 8x2 and A*B=0.

Use four optimization variables z=[v(2); x(2)] and:

    u(v,x) = P_f*F*d(v) + u_moment + M*v + B*x

This lets the allocator redistribute load without changing the requested wrench. A faithful extension of the FA code's reduced objective can add a null-space cost:

    J = alpha/2*||v-v_ref||^2 + beta/2*||x||^2

To include the paper's smoothing objective, use:

    J += gamma/2*||u(v,x)-u_prev||^2

Here B is orthonormal; otherwise use x.transpose()*(B.transpose()*B)*x. Choose positive alpha and beta, and gamma>=0 with attention to command units. With gamma=0 this omits smoothing like the source. With gamma>0 it is an intentional extension toward Eq. (18). Finite weighted costs trade attitude tracking against allocation/smoothing; they do not impose strict lexicographic priority.

Constraints for either variant:

    u_min[i] <= u_i(v,x) <= u_max[i], i=1,...,8
    ||v||^2 <= r^2

For source parity, r=||clipped v_ref|| and use that clipped reference in J. For the paper's reference handling, use the raw v_ref in J and r=min(||v_ref_raw||, max_v). The resulting optima can differ. There are 17 scalar inequalities before any additional platform constraints.

If actual command-rate limits are required, impose those separately using the control period; a smoothing penalty alone does not enforce them. Define u_prev consistently as the previous issued command in the same units, and keep estimated actuator state separate.

## Derivatives for a nonlinear solver

Within the valid domain s>0, define G=C.transpose()*C:

    Dd_Dv = C - d_B*(G*v).transpose()/s               // 3x2
    Du_Dz = [P_f*F*Dd_Dv + M, B]                     // 8x4
    grad_J = [alpha*(v-v_ref); beta*x]
             + gamma*Du_Dz.transpose()*(u-u_prev)

Use c<=0 convention:

    c_upper = u-u_max;       Jacobian = Du_Dz
    c_lower = u_min-u;       Jacobian = -Du_Dz
    c_step = v.transpose()*v-r^2; Jacobian = [2*v.transpose(), 0, 0]

These Jacobians use one constraint per row; MATLAB's supplied gradient array uses the transpose convention. Verify the chosen C++ solver's convention. With gamma>0 the composed objective is generally nonlinear too.

Do not copy the source's out-of-domain branch literally: it normalizes v, sets s=0, and then divides by s in the gradient. Use a solver-compatible domain treatment so trial evaluations remain finite and mathematically consistent. The feasible radius must stay strictly below one. Handle r=0 by fixing v=0 and solving only allocation rather than relying on the degenerate squared-norm constraint.

Use a nonlinear constrained optimizer or implement SQP with explicit linearization, trust-region/domain handling, and feasibility verification. A QP-only solver cannot accept this formulation directly. An SQP solve is local; do not claim global optimality from solver success.

## Suggested integration contract and loop

    configure(A, motor_bounds, mass, inertia, gains, conventions, solver_limits)
    step(state, reference, dt, previous_command) ->
        {motor_command, accepted, solver_status, max_constraint_violation,
         solve_time, attitude_step, null_coordinates}

At configuration: validate rank, units and mapping; precompute P_f, P_m, B. Recompute them if A changes. At each step: compute feedback, directions/basis, nonlinear map, solve, independently evaluate final feasibility and finiteness, then convert accepted u into the actual motor API's units.

If warm-starting, tangent coordinates from the previous cycle cannot simply be reused when C changes; project the previous physical direction into the new basis or reset v. A constant B permits reuse of null coordinates.

Set a solve deadline within the actual control budget. Route timeout, infeasibility, bad sensor input, or unacceptable residuals to the existing flight stack's defined fallback. Neither blindly sending the last iterate nor blindly clipping motor outputs preserves the requested wrench. The original block logs exitflag but has no explicit acceptance/fallback logic.

## Validation before platform integration

1. Compare the C++ source-parity implementation against captured MATLAB inputs/outputs, including force, moment baseline, basis, clipped reference, constraints, objective, and final command. Compare physical commands/directions rather than basis coordinates if the basis differs.
2. Check A*P_f, A*P_m, A*B, rank and all frame/motor signs. Exercise hover and each wrench axis independently.
3. Compare analytic gradients against numerical differences at interior points.
4. For the eight-motor extension, test cases where x=0 violates bounds but a null-space redistribution is feasible; confirm that B*x leaves wrench unchanged.
5. Exercise saturation, infeasible wrench requests, zero/small force, r=0, basis changes, large attitude differences, asymmetric limits and solver timeout.
6. Measure worst observed execution time and verify the acceptance/fallback path in software and hardware-in-the-loop before enabling motor authority. Use the real inertia and actuator model, not the FA simulation constants.

The custom allocation matrix and platform contract remain required inputs. No C++ controller has been implemented or flight-validated by this handoff.
