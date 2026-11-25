%%
clear; clc; close all;

%%
% Load the AERO 2 Parameters
aero2_parameters;
% Load stiffness, damping, and thrust parameters values
aero2_parameters_id;

%% Create state-space model in MATLAB
A = [0,         0,       1,       0;
    0,         0,       0,       1;
    -Ksp/Jp,   0,  -Dp/Jp,       0;
    0,         0,       0,  -Dy/Jy];

B = [0,              0;
    0,              0;
    Kpp*Dt/Jp,  Kpy*Dt/Jp;
    Kyp*Dt/Jy,  Kyy*Dt/Jy];

C = [1, 0, 0, 0;
    0, 1, 0, 0];

D = zeros(2, 2);

aero2_ss = ss(A,B,C,D)

disp('A = ')
disp(A);
disp('B = ')
disp(B)

% open-loop poles:
eig(aero2_ss)

Co = ctrb(A, B);
Ob = obsv(A, C);

fprintf('Rank Controllability: %d\n', rank(Co));
fprintf('Rank Observability: %d\n', rank(Ob));