% Load parameters
qube2_param;

% State-space matrices
A = [...];
B = [...];
C = [...];
D = [...];

%simulate a step to the state space model
motor_ss=ss(A,B,C,D);
% step(motor_ss,5);