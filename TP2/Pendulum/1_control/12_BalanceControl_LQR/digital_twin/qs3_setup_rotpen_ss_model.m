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
% Polos em malha aberta (autovalores da matriz A)
poles_open = eig(A);
% Display the open-loop poles
disp('Open-loop poles of the inverted pendulum:');
disp(poles_open);
if all(real(poles_open) < 0)
    disp('The system is stable.');
else
    disp('The system is unstable.');
end

%% controllability and observability
% Verificacao de controlabilidade e observabilidade
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


%% Control synthesis: LQR (Questao 12)
% Requisitos: |alpha| < 3 deg e Overshoot da base (theta) < 14%
% Matrizes de ponderacao Q e R sintonizadas:
% - q11 = 10 acelera o rastreamento do braco (theta)
% - q44 = 5 amortece a velocidade do pendulo (alpha_dot) e limita o desvio |alpha|
Q = diag([10 1 1 5]);
R = 1;

% Calculo do ganho otimo LQR
K = lqr(A, B, Q, R);

% Verificacao dos polos em malha fechada (A - B*K)
poles_closed = eig(A - B*K);
disp('Closed-loop poles (LQR):');
disp(poles_closed);

% Exibicao da matriz de ganho K
disp('State-feedback gain K (LQR):');
disp(K);

%% ========================================================================
%  QUESTOES 12 e 13: Simulacao Linear em Malha Fechada (lsim)
%  ========================================================================
% Mesmo sinal da Questao 9: onda quadrada de +-30 deg a 0.125 Hz (3 ciclos)
t = 0:0.001:24;
theta_d_deg = 30 * sign(sin(2 * pi * 0.125 * t));
theta_d_deg(1) = 30;
theta_d_rad = deg2rad(theta_d_deg);

% Malha fechada com referencia na posicao do braco: u = -K*x + K(1)*theta_d
Acl = A - B * K;
Bcl = B * K(1);
sys_cl = ss(Acl, Bcl, eye(4), zeros(4, 1));

% Simulacao linear a partir do equilibrio vertical (x0 = 0)
x = lsim(sys_cl, theta_d_rad, t);
theta_deg = rad2deg(x(:, 1));
alpha_deg = rad2deg(x(:, 2));
u_volts   = -K * x' + K(1) * theta_d_rad;

% Graficos de resposta (theta, alpha e tensao do motor)
figure('Color', 'w', 'Name', 'Questoes 12 e 13 - LQR Balance Control');

subplot(3,1,1);
plot(t, theta_d_deg, 'k--', 'LineWidth', 1.2); hold on;
plot(t, theta_deg, 'b', 'LineWidth', 1.5);
grid on; ylabel('\theta (deg)');
legend('Referencia', 'Braco (theta)');
title('Rotary Arm Angle (deg)');

subplot(3,1,2);
plot(t, alpha_deg, 'b', 'LineWidth', 1.5); hold on;
yline(3, 'r--'); yline(-3, 'r--');
grid on; ylabel('\alpha (deg)');
title('Pendulum Angle Deviation from Upright (deg)');

subplot(3,1,3);
plot(t, u_volts, 'r', 'LineWidth', 1.2);
grid on; xlabel('Tempo (s)'); ylabel('V_m (V)');
title('Motor Command Voltage (V)');

% Metricas de desempenho no primeiro degrau (0 -> 30 deg) e na onda completa
info = stepinfo(theta_deg(1:4000), t(1:4000), 30);
alpha_max_step = max(abs(alpha_deg(1:4000)));
alpha_max_full = max(abs(alpha_deg));

fprintf('\n--- Desempenho Pendulo - LQR (Q12-13) ---\n');
fprintf('Base (theta)    -> Overshoot (PO): %.2f %% (< 14%%) | Settling Time (ts): %.3f s\n', ...
    info.Overshoot, info.SettlingTime);
fprintf('Pendulo (alpha) -> Max |alpha| (0->30 deg): %.2f deg | Max |alpha| (ciclo completo): %.2f deg (< 3 deg)\n', ...
    alpha_max_step, alpha_max_full);



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
