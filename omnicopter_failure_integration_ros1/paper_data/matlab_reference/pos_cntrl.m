%% %%%%%%%%%%%%%%%%%%% pos_cntrl %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function computes desired force to achieve desired position (with
% corresponding derivatives)
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [f, e_pR, e_vR, mge3] = pos_cntrl(...
    pR_d, dpR_d, ddpR_d,... % desired trajectory
    pR, dpR, e_pR_I,  ... % state
    mR, g, ... % parameters system
    Kp_p, Kd_p, Ki_p, e_pR_sat, e_dpR_sat)  % parameters controller

% general variables
e3 = [0 0 1]';

e_pR = pR_d - pR;
e_vR = dpR_d - dpR;

e_pR = sign(e_pR).*min(e_pR_sat*ones(3,1),abs(e_pR));
e_vR = sign(e_vR).*min(e_dpR_sat*ones(3,1),abs(e_vR));

% force 
f = (mR*ddpR_d + Kp_p*e_pR...
    + Kd_p*e_vR + Ki_p*e_pR_I) + mR*g*e3;  % desired force

mge3 = mR*g*e3;


