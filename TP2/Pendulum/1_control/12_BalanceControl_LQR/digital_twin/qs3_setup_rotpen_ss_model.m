%% 
clear; clc; close all;

%% calculate state-space model matrices

% Load QUBE-Servo 3 model parameters
qs3_rotpen_param;
% Load state-space model for pendulum in upright configuration
rotpen_ABCD_eqns_ip;
% Create Matlab state-space object for analysis
sys = ss(A,B,C,D);

%% open-loop poles of inverted pendulum
% Hint: eigenvalues of matrix A: eig(A) 
poles_open = ...;
% Display the open-loop poles
disp('Open-loop poles of the inverted pendulum:');
disp(poles_open);
if all(poles_open<0)
    disp('The system is stable.');
else
    disp('The system is unstable.');
end

%% controllability and observability
% Hint: using ctrb() and obsv() function in MATLAB
% Co = ctrb(...);
% Ob = obsv(...);


% Rank of controllability and observability matrices
disp('Rank of controllability matrix:');
disp(rank(Co));
disp('Rank of observability matrix:');
disp(rank(Ob));

% Check controllability and observability
if rank(Co) == size(A, 1)
    disp('The system is controllable.');
else
    disp('The system is not controllable.');
end

% Check observability
if rank(Ob) == size(A, 1)
    disp('The system is observable.');
else
    disp('The system is not observable.');
end


%% control requirements 
% PO_max = 6.81;  % percentage
% t_s_max = 1.54; % seconds


%% Control synthesis: LQR
% Find Q, R and K matrices
% LQR Weighting Matrices
Q = diag([1 1 1 1]);
R = 1;
K = lqr(A,B,Q,R);
% Verify the stability of the closed-loop system with state feedback
poles_closed = eig(A - B*K);
disp('Closed-loop poles:');
disp(poles_closed);
% Display state-feedback gain K
disp('State-feedback gain K:');
disp(K);

% Choose which states are base angle (theta) and pendulum angle (alpha)
i_theta = 1;   % index of base angle in x
i_alpha = 2;   % index of pendulum angle in x

% --- Reference (degrees -> radians) ---
theta_ref_deg = 30;
alpha_ref_deg = 0;
x_ref = zeros(size(A,1),1);
x_ref(i_theta) = deg2rad(theta_ref_deg);
x_ref(i_alpha) = deg2rad(alpha_ref_deg);

% --- Compute steady-state input u_ss so that A * x_ref + B * u_ss = 0 ---
% use least-squares in case B is not square
u_ss = - pinv(B) * (A*x_ref);
resid = norm(A*x_ref + B*u_ss);
if resid > 1e-6
    warning('x_ref is not an exact equilibrium (||A x_ref + B u_ss||=%.2e). Consider integral action.', resid);
end

% --- Error system: e = x - x_ref;  e_dot = (A - B K) e ---
Acl = A - B*K;
nx = size(A,1);

% initial state (change if needed)
% x0 = zeros(nx,1);
x0 = [deg2rad(-30), deg2rad(0), 0, 0]';
e0 = x0 - x_ref;

% simulate zero-input initial response of e(t)
t = linspace(0, 5, 2000).';
sys_e = ss(Acl, zeros(nx,1), eye(nx), zeros(nx,1));
[e, t] = initial(sys_e, e0, t);

% reconstruct states and input
x = e' + x_ref;
u = (-K*e.' + u_ss).';   % m x N -> N x m

theta = x(i_theta, :);   % radians
alpha = x(i_alpha, :);   % radians


%% --- Plots ---
figure('Color','w','Name','LQR State-Tracking (x_{ref})');
subplot(2,1,1);
plot(t, theta*180/pi, 'LineWidth', 1.6); hold on; grid on;
yline(theta_ref_deg,'--k','Reference');
xlabel('Time (s)'); ylabel('\theta (deg)');
title(sprintf('\\theta tracking '));

subplot(2,1,2);
plot(t, alpha*180/pi, 'LineWidth', 1.6); hold on; grid on;
yline(alpha_ref_deg,'--k','Reference');
xlabel('Time (s)'); ylabel('\alpha (deg)');
title(sprintf('\\alpha tracking  '));



%% Observation => Tuning procedure
% - changing the q11 element primarily increases the rotary arm proportional
% gain
% - increasing q11 augments the gain on rotary arm => responds faster, 
% the pendulum tends to deflect more from its vertical position
% - changing q22 has little affect on the generated gain K
% - q44 is the main variable to adjust.

% The procedure to find out which Q element affect the pendulum 
% the most effectively is as follows:

% i. Adjust Q matrix element.
% ii. Compute the gain K.
% iii. Verify if the new gain has changed significantly. 
% If not, then adjust another element or increase more. 
% If it has changed, go to next step.
% iv. Implement the gain on QUARC controller
% v. Examine the pendulum response
