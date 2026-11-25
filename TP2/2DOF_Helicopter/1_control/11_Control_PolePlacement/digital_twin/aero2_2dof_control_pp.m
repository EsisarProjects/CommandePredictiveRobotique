%%
clear; clc; close all;

%%
% Load the AERO 2 Parameters
aero2_parameters;
% Load stiffness, damping, and thrust parameters values
aero2_parameters_id;

%% Create state-space model in MATLAB
% Matrizes do modelo linearizado do Aero 2: estado x = [theta; psi; theta_dot; psi_dot]
% Entradas u = [Vp; Vy] e saidas y = [theta; psi] (Dt = braco de alavanca dos rotores)
A = [0,       0,      1,      0;
     0,       0,      0,      1;
     -Ksp/Jp, 0, -Dp/Jp,      0;
     0,       0,      0, -Dy/Jy];

B = [0,            0;
     0,            0;
     Kpp*Dt/Jp, Kpy*Dt/Jp;
     Kyp*Dt/Jy, Kyy*Dt/Jy];

C = [1, 0, 0, 0;
     0, 1, 0, 0];

D = zeros(2, 2);
aero2_ss = ss(A,B,C,D);

% Polos em malha aberta
disp('Polos em malha aberta (Aero 2):');
disp(eig(aero2_ss));

%% State-Feedback Pole Placement Control Design
% Especificacoes da Questao 10: PO <= 5%, tp_pitch <= 2.5 s, tp_yaw <= 3.5 s
PO_p_max = 5; % percentage
PO_y_max = PO_p_max;
tp_p_max = 2.5; % seconds
tp_y_max = 3.5;

% Valores de projeto ajustados (mais restritivos para compensar o acoplamento MIMO e a saturacao)
PO_p = 3;
PO_y = 3;
tp_p = 1.8;
tp_y = 2.0;

% Calculo de zeta, wd, wn e polos desejados para Pitch (theta)
zeta_p = -log(PO_p/100) / sqrt(pi^2 + log(PO_p/100)^2);
wd_p   = pi / tp_p;
wn_p   = wd_p / sqrt(1 - zeta_p^2);
p1_p   = -zeta_p * wn_p + 1i * wd_p;
p2_p   = -zeta_p * wn_p - 1i * wd_p;

% Calculo de zeta, wd, wn e polos desejados para Yaw (psi)
zeta_y = -log(PO_y/100) / sqrt(pi^2 + log(PO_y/100)^2);
wd_y   = pi / tp_y;
wn_y   = wd_y / sqrt(1 - zeta_y^2);
p1_y   = -zeta_y * wn_y + 1i * wd_y;
p2_y   = -zeta_y * wn_y - 1i * wd_y;

% Equacao caracteristica desejada a partir dos 4 polos
desired_poles = [p1_p, p2_p, p1_y, p2_y];
char_eq = poly(desired_poles);

% Display the characteristic equation
disp('Desired characteristic equation coefficients:');
disp(char_eq);

% Calculo da matriz de ganho K por alocacao de polos
K = place(A, B, desired_poles);

% Verificacao da estabilidade em malha fechada (A - B*K)
poles_closed = eig(A - B*K);
disp('Closed-loop poles:');
disp(poles_closed);

% Exibicao da matriz de ganho K
disp('State-feedback gain K:');
disp(K);


%% ========================================================================
%  QUESTOES 10 e 11: Simulacao em Malha Fechada (com saturacao +-24V)
%  ========================================================================
t = 0:0.01:30;

% Sinais de referencia em onda quadrada (iguais ao modelo Simulink):
% Pitch: +-20 deg a 0.05 Hz | Yaw: +-45 deg a 0.04 Hz
theta_d_deg = 20 * sign(sin(2 * pi * 0.05 * t));
theta_d_deg(1) = 20;
psi_d_deg   = 45 * sign(sin(2 * pi * 0.04 * t));
psi_d_deg(1) = 45;

% Funcao de referencia instantanea xd(t) e lei de controle com saturacao +-24V
xd_fun = @(tk) [deg2rad(20 * (2*(mod(tk, 20) < 10) - 1)); ...
                deg2rad(45 * (2*(mod(tk, 25) < 12.5) - 1)); 0; 0];
u_fun  = @(tk, xk) max(-24, min(24, K * (xd_fun(tk) - xk)));

% Integracao da dinamica x_dot = A*x + B*u
[~, x] = ode45(@(tk, xk) A*xk + B*u_fun(tk, xk), t, zeros(4,1));

theta_deg = rad2deg(x(:, 1));
psi_deg   = rad2deg(x(:, 2));
u = zeros(length(t), 2);
for i = 1:length(t)
    u(i, :) = u_fun(t(i), x(i, :)')';
end
Vp = u(:, 1);
Vy = u(:, 2);

% Graficos de resposta (angulos e tensoes dos rotores)
figure('Color', 'w', 'Name', 'Questoes 10 e 11 - Aero 2 Pole Placement');
subplot(2,2,1);
plot(t, theta_d_deg, 'k--', 'LineWidth', 1.2); hold on;
plot(t, theta_deg, 'b', 'LineWidth', 1.5);
grid on; title('Pitch Angle'); ylabel('pitch (deg)');
legend('Referencia', 'Pitch');

subplot(2,2,2);
plot(t, psi_d_deg, 'k--', 'LineWidth', 1.2); hold on;
plot(t, psi_deg, 'b', 'LineWidth', 1.5);
grid on; title('Yaw Angle'); ylabel('yaw (deg)');
legend('Referencia', 'Yaw');

subplot(2,2,3);
plot(t, Vp, 'r', 'LineWidth', 1.2);
ylim([-28 28]);
grid on; title('Pitch Rotor/Motor Voltage'); xlabel('Tempo (s)'); ylabel('V_p (V)');

subplot(2,2,4);
plot(t, Vy, 'r', 'LineWidth', 1.2);
ylim([-28 28]);
grid on; title('Yaw Rotor/Motor Voltage'); xlabel('Tempo (s)'); ylabel('V_y (V)');
sgtitle('Aero 2 - Pole Placement Control');

% Metricas de desempenho no primeiro degrau (0 a 9.9 s, antes da inversao do pitch)
idx = t <= 9.9;
info_p = stepinfo(theta_deg(idx), t(idx));
info_y = stepinfo(psi_deg(idx), t(idx));

fprintf('\n--- Desempenho Aero 2 - Pole Placement (Q10-11) ---\n');
fprintf('Pitch -> Overshoot (PO): %.2f %% (<= 5%%) | Peak Time (tp): %.3f s (<= 2.5 s) | Settling Time (ts): %.3f s\n', ...
    info_p.Overshoot, info_p.PeakTime, info_p.SettlingTime);
fprintf('Yaw   -> Overshoot (PO): %.2f %% (<= 5%%) | Peak Time (tp): %.3f s (<= 3.5 s) | Settling Time (ts): %.3f s\n', ...
    info_y.Overshoot, info_y.PeakTime, info_y.SettlingTime);