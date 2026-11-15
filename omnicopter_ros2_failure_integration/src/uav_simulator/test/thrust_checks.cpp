#include "../src/thrust_dynamics.hpp"
#include <cmath>
#include <stdexcept>
#include <iostream>
int main(){
 auto check=[](bool v){if(!v)throw std::runtime_error("Actuator dynamics mismatch");};
 const double initial=10.;
 check(std::abs(omni_sim::advanceThrust(initial,0.,.21,.15,.21)-initial/std::exp(1.))<1e-12);
 check(std::abs(omni_sim::advanceThrust(0.,initial,.15,.15,.21)-initial*(1.-1./std::exp(1.)))<1e-12);
 const double residual=omni_sim::advanceThrust(initial,0.,.0025,.15,.21);
 check(residual>9.&&residual<initial); // Cap target to zero, preserve residual physical thrust.
 double a=initial;
 for(int i=0;i<84;i++)a=omni_sim::advanceThrust(a,0.,.0025,.15,.21);
 check(std::abs(a-initial/std::exp(1.))<1e-12); // Time-step independent ZOH.
 check(omni_sim::advanceThrust(initial,0.,0.,.15,.21)==initial);
 std::cout<<"PASS: acceleration, zero-braking decay, residual thrust and timestep invariance\n";
}
