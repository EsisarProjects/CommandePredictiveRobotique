%% Exercise 2.2 - UAV Trajectory with Constant Velocity Va
clear; clc; close all;

% 1. System parameters
param.g = 9.8;
param.h = 0.05;

% 2. Initial and final conditions
Va = 20;                % Constant speed[m/s]
xi = [0; 0; 0];         % [x0; y0; psi0]
phii = 0;               % Initial Roll [rad]

psif = pi/4;            % Desired final guidance [rad]
phif = 0;               % Final Roll [rad]
T = 10;                 % Total time [s]

% Angular rate of change (psi_dot) at the ends: psi_dot = g*tan(phi)/Va
psid_i = (param.g * tan(phii)) / Va;
psid_f = (param.g * tan(phif)) / Va;

% 3. 4x4 linear system for the cubic polynomial of Psi(t) = c1 + c2*t + c3*t^2 + c4*t^3
M_psi = [ 1,  0,    0,      0; ...
    0,  1,    0,      0; ...
    1,  T,  T^2,    T^3; ...
    0,  1,  2*T,  3*T^2 ];

b_psi = [ xi(3); psid_i; psif; psid_f ];

c = M_psi \ b_psi;

% 4. Functions for Psi(t) and its derivative Psi_dot(t)
psi_fun  = @(t) ( c(1) + c(2)*t + c(3)*t.^2 + c(4)*t.^3 );
psid_fun = @(t) ( c(2) + 2*c(3)*t + 3*c(4)*t.^2 );

% 5. Time evaluation and numerical integration of position (x, y)
tt = 0 : param.h : T;
N = length(tt);

psi_ref = psi_fun(tt);
psid_ref = psid_fun(tt);

% Roll angle Phi(t): tan(Phi) = (Va * psi_dot) / g
phi_ref = atan( (Va * psid_ref) / param.g );
Va_ref  = Va * ones(1, N);

% Euler/Trapezoidal Integration for x(t) e y(t)
x_ref = zeros(1, N);
y_ref = zeros(1, N);
x_ref(1) = xi(1);
y_ref(1) = xi(2);

for k = 1:N-1
    x_ref(k+1) = x_ref(k) + param.h * Va * cos(psi_ref(k));
    y_ref(k+1) = y_ref(k) + param.h * Va * sin(psi_ref(k));
end

% 6. Plotting the results
figure('Color', 'w');

subplot(2,2,1);
plot(x_ref, y_ref, 'b', 'LineWidth', 2);
grid on; xlabel('x [m]'); ylabel('y [m]'); title('Trajectory (x, y) with V_a constant');

subplot(2,2,2);
plot(tt, psi_ref, 'r', 'LineWidth', 1.5);
grid on; xlabel('t [s]'); ylabel('\Psi [rad]'); title('Orientation \Psi(t)');

subplot(2,2,3);
plot(tt, Va_ref, 'g', 'LineWidth', 2);
ylim([15 25]);
grid on; xlabel('t [s]'); ylabel('V_a [m/s]'); title('Constant Velocity V_a');

subplot(2,2,4);
plot(tt, phi_ref, 'm', 'LineWidth', 1.5);
grid on; xlabel('t [s]'); ylabel('\Phi [rad]'); title('Roll Angle \Phi(t)');
