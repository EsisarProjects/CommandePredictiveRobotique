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
poles_open = eig(A);

% Display the open-loop poles
disp('Open-loop poles of the inverted pendulum:');
disp(poles_open);
if all(poles_open < 0)
    disp('The system is stable.');
else
    disp('The system is unstable.');
end

%% controllability and observability
% Hint: using ctrb() and obsv() function in MATLAB
Co = ctrb(A, B);
Ob = obsv(A, C);

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

%% Control synthesis: Pole placement
% Find dominant poles corresponding to specifications:
% PO < 6.81%
% t_settling < 1.54s

PO_max = 6.81;  % percentage
t_s_max = 1.54; % seconds
PO = 6.5;       % chosen overshoot    
t_s = 1.5;      % chosen settling time
zeta = -log(PO/100) / sqrt( pi^2 + log(PO/100)^2);
wn = 4 / (t_s * zeta);

% display natural frequency and damping ratio:
disp(['Natural frequency (wn): ', num2str(wn), ' (rad/s)']);
disp(['Damping ratio (zeta): ', num2str(zeta)]);

% two dominant poles p1 and p2:
sigma = - zeta * wn;
wd = wn * sqrt(1 - zeta^2);
p1 = sigma + 1i*wd; 
p2 = sigma - 1i*wd; 

% display dominant poles p1 and p2:
disp('Dominant poles of the system:');
disp([p1, p2]);

% Find the desired characteristic equation 
% if the other poles are placed at p3 = -40 and p4 = -45
p3 = -40;
p4 = -45;

% Calculate the desired characteristic equation coefficients
desired_poles = [p1, p2, p3, p4];
char_eq = poly(desired_poles);

% Display the characteristic equation
disp('Desired characteristic equation coefficients:');
disp(char_eq);

% calculate state-feedback gain K
K = place(A, B, desired_poles);

% Verify the stability of the closed-loop system with state feedback
poles_closed = eig(A - B*K);
disp('Closed-loop poles:');
disp(poles_closed);

% Display state-feedback gain K
disp('State-feedback gain K:');
disp(K);

%% ========================================================================
%  QUESTION 9: Simulação em Malha Fechada (Degrau de +-30 deg a 0.125 Hz)
%  ========================================================================
t = 0:0.001:24; % 3 ciclos completos (T = 1/0.125 = 8 s)

% Referência de posição do braço theta_d: onda quadrada de +-30 graus a 0.125 Hz
theta_d_deg = 30 * sign(sin(2 * pi * 0.125 * t));
theta_d_rad = deg2rad(theta_d_deg);

% Malha fechada com referência na posição do braço: u = -K*x + K(1)*theta_d
Acl = A - B * K;
Bcl = B * K(1);
sys_cl = ss(Acl, Bcl, eye(4), zeros(4, 1));

% Simulação da resposta a partir do equilíbrio vertical (x0 = 0)
x = lsim(sys_cl, theta_d_rad, t);

theta_deg = rad2deg(x(:,1));
alpha_deg = rad2deg(x(:,2));
u_volts   = -K * x' + K(1) * theta_d_rad;

figure('Color', 'w', 'Name', 'Questão 9 - PD Balance Control');

subplot(3,1,1);
plot(t, theta_d_deg, 'k--', 'LineWidth', 1.2); hold on;
plot(t, theta_deg, 'b', 'LineWidth', 1.5);
grid on; ylabel('\theta (deg)');
legend('Setpoint', 'Rotary Arm');
title('Rotary Arm Angle (deg)');

subplot(3,1,2);
plot(t, alpha_deg, 'b', 'LineWidth', 1.5);
grid on; ylabel('\alpha (deg)');
title('Pendulum Angle Deviation from Upright (deg)');

subplot(3,1,3);
plot(t, u_volts, 'r', 'LineWidth', 1.2);
grid on; xlabel('Tempo (s)'); ylabel('V_m (V)');
title('Motor Command Voltage (V)');

% Métricas de desempenho no primeiro degrau (0 -> 30 deg)
info = stepinfo(theta_deg(1:4000), t(1:4000), 30);
fprintf('\n--- Desempenho do Braço (Degrau 0 -> 30 deg) ---\n');
fprintf('Overshoot (PO): %.2f %% (Requisito: <= 6.81 %%)\n', info.Overshoot);
fprintf('Settling Time (ts): %.3f s (Requisito: <= 1.54 s)\n', info.SettlingTime);