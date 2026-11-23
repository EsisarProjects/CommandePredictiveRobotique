%% Validação do Modelo Linear - Questão 3 (Onda Quadrada Simétrica 1 Hz, 20 s)
qs3_setup_rotpen_ss_model;

% Vetor de tempo de 0 a 20 segundos
t = 0:0.001:20;

% Onda quadrada de 1 Hz alternada [-1 V a +1 V] usando funções base do MATLAB
u = -sign(sin(2 * pi * 1 * t));

% Condição inicial: pêndulo em repouso apontado para baixo
x0 = [0; 0; 0; 0];

% Simulação do modelo linear no espaço de estados
y = lsim(sys, u, t, x0);

% Gráficos comparativos
f = figure(10);
clf(f);
set(f, 'Name', 'Q3 - State Space Response', 'Color', 'w');

subplot(2,1,1);
plot(t, y(:,1), 'b', 'LineWidth', 1.5);
grid on;
ylabel('\theta (rad)');
title('Base Encoder Angle (rad) - Resposta do Modelo Linear');

subplot(2,1,2);
plot(t, y(:,2), 'b', 'LineWidth', 1.5);
grid on;
xlabel('Tempo (s)');
ylabel('\alpha (rad)');
title('Pendulum Angle (rad) - Resposta do Modelo Linear');

drawnow;
shg;