%%
clear; clc; close all;

%%
% Load the AERO 2 Parameters
aero2_parameters;
% Load stiffness, damping, and thrust parameters values
aero2_parameters_id;

%% Create state-space model in MATLAB
A = ...;
B = ...;
C = ...;
D = ...;
aero2_ss = ss(A,B,C,D)

disp('A = ')
disp(A);
disp('B = ')
disp(B)

% open-loop poles:
eig(aero2_ss)