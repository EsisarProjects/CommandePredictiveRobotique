% - Qube servo - DC motor control (MPC with CasADi)

%%
clc; clear; close all;

%%
% Load parameters
qube3_param;

% Continuous system (modelo identificado na Q6: K1 = 24.5, tau1 = 0.08)
K1 = 24.5;
tau1 = 0.08;

A_cont = [0, 1;
          0, -1/tau1];
B_cont = [0;
          K1/tau1];
C_cont = [1, 0;
          0, 1];
D_cont = [0;
          0];

% System dimensions
[dx, du] = size(B_cont);
dy = size(C_cont, 1);

% Discretize the system
import casadi.*

Ts = 0.02; % Sampling time
sys_cont = ss(A_cont, B_cont, C_cont, D_cont); % Create continuous state-space system
sys_disc = c2d(sys_cont, Ts, 'zoh'); % Discretize the system using zero-order hold

[A, B, C, D] = ssdata(sys_disc);

% Initial conditions
x0 = [0; 0];
u0 = zeros(du, 1);

% Constraints (baseado na Eq. 18: |theta| <= 1 rad, |u| <= 5 V)
umin = -5;
umax = 5;
theta_min = -1;
theta_max = 1;

% Weighting matrices (Regra de Bryson da Q7 + custo terminal P de Riccati discreta)
omega_max = 5 / km;
Q = diag([1 / theta_max^2, 1 / omega_max^2]);
R = 1 / umax^2;
[~, P, ~] = dlqr(A, B, Q, R); % Matriz terminal P (solucao da ARE discreta)

% Horizons
Npred = 20; % Horizonte de predicao (20 passos * 0.02s = 0.40s)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%=======================================
solver = casadi.Opti();

% Variables and Parameters
x = solver.variable(dx, Npred+1);
u = solver.variable(du, Npred);
xinit = solver.parameter(dx, 1);
uinit = solver.parameter(du, 1);
xref = solver.parameter(dx, 1);

% Constraints
solver.subject_to(x(:,1) == xinit);
for k = 1:Npred
    solver.subject_to(x(:,k+1) == A * x(:,k) + B * u(:,k));
    solver.subject_to(umin <= u(:,k));
    solver.subject_to(u(:,k) <= umax);
    solver.subject_to(theta_min <= x(1,k+1));
    solver.subject_to(x(1,k+1) <= theta_max);
end

% Cost
objective = 0;
for k = 1:Npred
    objective = objective + (x(:,k) - xref)' * Q * (x(:,k) - xref) + u(:,k)' * R * u(:,k);
end
objective = objective + (x(:,Npred+1) - xref)' * P * (x(:,Npred+1) - xref);
solver.minimize(objective);

% Solver options
opts = struct('qpsol', 'qrqp', 'print_time', false);
solver.solver('sqpmethod', opts);

%%
Tfinal = 10;
t = 0:Ts:Tfinal;
t = t(1:end-1);
Nsim = length(t);
% Reference (tracking only theta - sem depender do Signal Processing Toolbox)
ref1 = -(pi/4) * sign(sin(2*pi*0.5*t));
ref1(1) = -pi/4;
ref2 = zeros(size(t));
ref = [ref1; ref2];

%% Simulation loop
xsim = zeros(dx, Nsim+1);
xsim(:,1) = x0;
usim = zeros(du, Nsim);
ysim = zeros(dy, Nsim);
usiminit = u0;

timer = tic;
for i = 1:Nsim
    xref_val = ref(:,i);
    solver.set_value(xinit, xsim(:,i));
    solver.set_value(xref, xref_val);
    solver.set_value(uinit, usiminit);

    sol = solver.solve();
    u_mpc = sol.value(u(:,1));

    usim(:,i) = u_mpc;
    xsim(:,i+1) = A*xsim(:,i) + B*u_mpc;
    ysim(:,i) = C*xsim(:,i) + D*u_mpc;

    solver.set_initial(sol.value_variables());
end
time_CasAdi = toc(timer);

%% Plotting
figure;
plot(t, ysim(1,:), 'b', 'LineWidth', 1.5); hold on;
plot(t, ref(1,:), 'r--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('\theta [rad]');
legend('Output', 'Reference');
title('Position Tracking (MPC)');

figure;
plot(t, usim, 'b', 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Control Input u [V]');
title('Control Input (MPC)');

%% Errors
error = abs(ysim - ref);
AvgError = mean(error, 2);
fprintf('Average error θ: %.4f rad\n', AvgError(1));
fprintf('Average control effort: %.4f V\n', mean(abs(usim)));

%% Time
fprintf("Total simulation time: %f seconds. \n", time_CasAdi);