%% 
clear; clc; close all;

%% calculate state-space model matrices

% Load QUBE-Servo 3 model parameters
qube2_rotpen_param;
% Load state-space model for pendulum in downward configuration
qs2_rotpen_ABCD_eqns_down_student;
% Create Matlab state-space object for analysis
sys = ss(A,B,C,D);
disp('A = ')
disp(A);
disp('B = ')
disp(B)