OCTOROTOR CONTROLLER HANDOFF

Start with CONTROLLER_HANDOFF.md. It explains the source controller and the
proposed C++ extension for a full-rank 6x8 allocation matrix.

matlab_reference/ contains the original three controller functions extracted
from simulation_full_QP_w_md_FA.slx, plus the original initializer and skew
utility. The extracted function text is unchanged. These are reference files,
not a standalone runnable simulation; init_FA.m depends on the original model
and additional files that are not included.

Give your Codex the following request together with this ZIP:

Read CONTROLLER_HANDOFF.md and matlab_reference/. Integrate this controller
into our existing C++ platform using the allocation matrix and hardware
conventions I provide. First inspect our platform interfaces and confirm
allocation rank, command units, frame conventions, motor limits, inertia,
control period and fallback behavior. Distinguish MATLAB parity from the
proposed octorotor extension. Implement and validate in simulation before
hardware activation. Do not copy the simulation gains or motor constants
as hardware defaults. Resolve the documented gyroscopic torque-sign issue.

The recipient still needs to supply the actual allocation matrix and platform
configuration. No C++ implementation or flight validation is included.
The paper PDF is not included; its title and equation references are in the
handoff. Local source paths in that document are provenance, not dependencies
that must exist on the recipient's computer.
