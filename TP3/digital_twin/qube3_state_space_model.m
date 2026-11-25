% Load parameters
qube3_param;

% State-space matrices
A = [0, 1;
    0, -(kt * km) / (Jeq * Rm)];

B = [0;
    kt / (Jeq * Rm)];

C = [1, 0;
    0, 1];

D = [0;
    0];

% simulate a step to the state space model
motor_ss = ss(A, B, C, D);
step(motor_ss, 5);