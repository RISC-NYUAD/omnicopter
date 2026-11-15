%% %%%%%%%%%%%%%%%%%%% att_cntrl %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This function computes desired force to achieve desired position (with
% corresponding derivatives)
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [m, e_Rr] = att_cntrl(...
    Rr, Rr_d, omegaR,... % state
    JR, Kp_R, Kd_R)  ... % parameters system  % parameters controller


% momentum
e_Rr_skew = -1/2*(Rr_d'*Rr - Rr'*Rr_d);   
e_Rr = [-e_Rr_skew(2,3) e_Rr_skew(1,3) -e_Rr_skew(1,2)]';   % rotation error

e_omegaR = - omegaR;%omegaR_d - omegaR;                               % angular velocity error
%m = (JR*domegaR_d + Kp_R*e_Rr + Kd_R*e_omegaR) + skew(JR*omegaR)*omegaR;  % desired momentum
m = (Kp_R*[0;0;e_Rr(3)] + Kd_R*e_omegaR) + skew(JR*omegaR)*omegaR;  % desired momentum

% input allocation
