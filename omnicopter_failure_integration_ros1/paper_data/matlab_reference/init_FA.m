%%%%%%%%%%%%%%%%%%%%% INIT %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Defines all the parameters needed for the simulations
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all
close all
addpath('./utilities');
addpath('./Plotting');
%% SIMULATION PARAMETES %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Tsim = 0.002;
g = 9.81;               % gravity (N)

tolerance = 0.01;
% noise
if_noise = 0;

% external disturbance
f_ext.time = [0,30,60,90,120];
f_ext.value = 0*[[0;0;1],[0;1;0],[0;2;0],[1;0;0]];
%f_ext.value = 2*f_ext.value/norm(f_ext.value);

%% Noise state %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
var_pR = 5*(0.01)^2*ones(3,1);
seed_pR = [150 151 152];

var_dpR = 5*(0.02)^2*ones(3,1);
seed_dpR = [350 351 352];

var_omegaR = 5*(0.1)^2*ones(3,1);
seed_omegaR = [550 551 552];

var_rpyR = 5*(3/180*pi)^2*ones(3,1);
seed_rpyR = [650 651 652];


%% QUADROTOR PARAMETERS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% physical parameters

% real data
mR = 1.5;              % mass (Kg)
JR = 2*0.015*eye(3);      % inertia (Kg*m^2)

w_min = 00  ;        % [Hz]
w_max = 100;         % [Hz]

% Design parameters
n=6;

k_f  = 9.9016*10^(-4);                                                         % N/RPM^2
k_d  = 1.9*10^(-5);                                                         % Nm/RPM^2
%
u_max = w_max^2;
r = k_d/k_f;                                                               % lift force/drag moment
                                                         % length of arm

% Compute the allocation matrix -------------------------------------------
drag_sign = 1*(-1).^(1:n);                                                         % c = -1 1

% Star Shaped coplaner d
angles = (2*pi*(1:n)/n);

% plot force set flag
plot_force_set = 0;

A = 1.0e-03 *[...
    0.3624   -0.2476    0.2476   -0.3624   -0.6100    0.6100;...
   -0.4951    0.5614    0.5614   -0.4951   -0.0663   -0.0663;...
    0.7772    0.7772    0.7772    0.7772    0.7772    0.7772;...
    0.0070    0.5432    0.5432    0.0070   -0.5501   -0.5501;...
   -0.6312   -0.3216    0.3216    0.6312    0.3096   -0.3096;...
   -0.3812    0.3812   -0.3812    0.3812   -0.3812    0.3812];


% nominal data
mR_n = (1 + 0.0)*mR;
JR_n = (1 + 0.0)*JR;


R1 = axisrotation('z',angles(1));
R2 = axisrotation('z',angles(2));
R3 = axisrotation('z',angles(3));
R4 = axisrotation('z',angles(4));
R1T = R1';
R2T = R2';
R3T = R3';
R4T = R4';


nA_m = null(A(4:6,:));
nA_f = null(A(1:3,:));

pA_f = nA_m*pinv(A(1:3,:)*nA_m);
pA_m = nA_f*pinv(A(4:6,:)*nA_f);

pA = pinv(A);

delta = 0.01;
%% Motor model
%tauM = 0.05;
%ug_pinv = pinv(F)*[Rr0'*mR*g*[0 0 1]'; zeros(3,1)];
%u0 = ug_pinv + (w_min^2 - min(ug_pinv))*ones(size(F,2),1);

%% TRAJECTORY
% assumption rotate about a circle
parameters.v_circ = 0.15; % circular velocity norm
parameters.r = 1; % radius of circle
w = parameters.r*parameters.v_circ;
t = 2*pi/w;
initial_height = 3;
final_height = 2;
parameters.pose = [initial_height 0 0;...
    initial_height 0 0;...
    final_height 0 -atan2(final_height - initial_height,parameters.r-0.2);...
    final_height 0 -atan2(final_height - initial_height,parameters.r-0.2);...
    initial_height 0 0 ; initial_height 0 0 ];           % initial position [z roll pitch] 
parameters.full_time = [0; 2.0*t/10;3.5*t/10; 6.5*t/10; 8*t/10;t];
parameters.initial_time = parameters.full_time(1);                 % initial position
parameters.duration = parameters.full_time(end);                     % duration
parameters.full_traj = zeros(6,length(parameters.full_time));
for i = 1:length(parameters.full_time)
    if i == 1
        [a,e,y] = RotMat2Spherical(axisrotation('y',parameters.pose(i,2))*axisrotation('x',parameters.pose(i,3)));
        parameters.full_traj(:,i) = [parameters.r, 0, parameters.pose(i,1),...
            parameters.pose(i,2), parameters.pose(i,3), 0]';
    end
    theta = w*parameters.full_time(i);
    parameters.full_traj(:,i) = [parameters.r * cos(theta), parameters.r * sin(theta), parameters.pose(i,1), ...
            parameters.pose(i,2), parameters.pose(i,3), -theta]';
end
parameters.initial_pos = parameters.full_traj(:,1);
parameters.final_pos = parameters.full_traj(:,end);
parameters.end_time = t;%parameters.duration;

pR0 = parameters.initial_pos(1:3)';         % initial position (m);
roll = parameters.initial_pos(5);
pitch = parameters.initial_pos(4);
yaw  = parameters.initial_pos(6);
Rr0 = axisrotation('z',yaw)*axisrotation('y',pitch)*axisrotation('x',roll);


%Rr0 = [1 0 0;
%       0 1 0;
%       0 0 1];         % initial orientation   %%0 0 1; 0 1 0; -1 0 0

f_0 = Rr0' * [0;0;mR*g];
m_0 = zeros(3,1);
u_0 = pA*[f_0;m_0];



%% HYERARCHICAL CONTROLLER %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Kp_p = diag([10 10 10]);
Kd_p = diag([8 8 8]);
Ki_p = 0*diag([5 5 5]);
Kp_R = 5.0*diag([2.5 2.5 0.3]); %diag([2.3 2.3 0.4]);1.4
Kd_R = 0.15*Kp_R;%0.15
Kd_R(3,3) = 0.1*Kp_R(3,3);
Ki_R = 0.0*diag([0.5 0.5 0.5]);
e_pR_sat = 0.5; 
e_dpR_sat = 0.5;
Kp_R_ = 20 * Kp_R;
Kp_R_(3,3) = 0;
e_rR_sat = 0.2;

%pA_m_v = pA_m*Kp_R*[0 1;-1 0; 0 0];
pA_m_v = pA_m*Kp_R*[1 0 0;0 1 0; 0 0 0];


%% Simulation Noise and Delay
%trajectory_feasibility;
motor_delay = 0.01/Tsim;

d_ = 0.02; %norm of fe  %% 0.2   0.02
c_ = 0.05;%1-2*0.25;%0.05; % norm of added null  
b_ = 0.5;%0.5;%0.05; % angle
a_ = 3.5;%0.25/15;%1; % norm of fe related control 0.25/12  %%%40   3.5
max_vr = 0.1;
rol_tol = 1.1;
weights = [a_,b_,c_,d_];
rot_tol = 0.3;
close all
correct_rotation = 1; % 1 for corrected, -1 for uncorrected
sim('simulation_full_QP_w_md_FA');
u_real_2 = u_actual;

test_plot_FA;


Rr_FA_circ = Rr;
pR_FA_circ = pR;
u_FA_circ = u_corrected;
u_FA_circ_a = u_actual;
e_FA_circ = e_Rr;

save('FA_circ_traj', 'Rr_FA_circ', 'pR_FA_circ', 'u_FA_circ', 'u_FA_circ_a', 'e_FA_circ', 'stime')