#pragma once
#include <cmath>

namespace omni_sim {
// Exact zero-order-hold solution of tau * d(actual)/dt + actual = target.
// Inputs: nonnegative thrust [N], dt >= 0 [s], positive time constants [s].
inline double advanceThrust(double actual, double target, double dt,
                            double tau_up, double tau_down) {
  const double tau=target>=actual?tau_up:tau_down;
  // expm1 retains accuracy for very small physics timesteps.
  return actual+(target-actual)*(-std::expm1(-dt/tau));
}
}
