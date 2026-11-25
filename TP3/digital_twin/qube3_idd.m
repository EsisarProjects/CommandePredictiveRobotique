% identification vs physic-based modeling (Offline / Sem Quanser)

%% 1. Physic-based transfer functions
qube3_state_space_model;   % Carrega motor_ss, C e D
close all;

G = tf(motor_ss);          % transfer function matrix
G2 = G(2);                 % Vm -> Omega
[z, p, k] = zpkdata(G, 'v');

[num2, den2] = tfdata(G2, 'v');
K2 = num2(end) / den2(end);
tau2 = den2(1) / den2(end);

fprintf("Gain (phys) K2 = %.4f (rad/s)/V\n", K2);
fprintf("Time constant tau2 = %.4f s\n", tau2);

%% 2. Identified transfer function (Valores da Figura 5 - Q5 e Q6)
% Degrau de 5 V em t = 1s -> velocidade estabiliza em ~122.5 rad/s (K1 = 122.5/5 = 24.5)
% Tempo para atingir 63.2% (77.4 rad/s) apos t = 1s -> tau1 = 0.08 s
K1 = 24.5;
tau1 = 0.08;

G1 = tf(K1, [tau1, 1]);

fprintf("Gain (ident) K1 = %.4f (rad/s)/V\n", K1);
fprintf("Time constant tau1 = %.4f s\n", tau1);

%% 3. State-space matrices from identified model (Question 6)
A1 = [0, 1;
    0, -1/tau1];
B1 = [0;
    K1/tau1];
C1 = C;
D1 = D;

motor_ss_ident = ss(A1, B1, C1, D1);

disp('A1 = '); disp(A1);
disp('B1 = '); disp(B1);