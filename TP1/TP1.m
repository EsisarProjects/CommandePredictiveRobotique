clear; clc; close all;

% 1. Initialization of system parameters
param.g = 9.8;
param.h = 0.05;
param.dbl_Alt = 150;

% 2. Initial and final conditions
xi = [0; 0; 0];
ui = [20; 0];
xf = [100; 100; pi/4];
uf = [20; 0];
T  = 10;

% =========================================================================
% PART A: CLASSICAL POLYNOMIAL (Exercise 2.1 - for comparison)
% =========================================================================
M = [ 1 0 0   0     0       0       0 0 0   0     0       0; ...
      0 1 0   0     0       0       0 0 0   0     0       0; ...
      0 0 2   0     0       0       0 0 0   0     0       0; ...
      0 0 0   0     0       0       1 0 0   0     0       0; ...
      0 0 0   0     0       0       0 1 0   0     0       0; ...
      0 0 0   0     0       0       0 0 2   0     0       0; ...
      1 T T^2 T^3   T^4     T^5     0 0 0   0     0       0; ...
      0 1 2*T 3*T^2 4*T^3   5*T^4   0 0 0   0     0       0; ...
      0 0 2   2*3*T 3*4*T^2 4*5*T^3 0 0 0   0     0       0; ...
      0 0 0   0     0       0       1 T T^2 T^3   T^4     T^5; ...
      0 0 0   0     0       0       0 1 2*T 3*T^2 4*T^3   5*T^4;...
      0 0 0   0     0       0       0 0 2   2*3*T 3*4*T^2 4*5*T^3];

b = [ xi(1) ui(1)*cos(xi(3)) -sin(xi(3))*param.g*tan(ui(2)) ...
      xi(2) ui(1)*sin(xi(3))  cos(xi(3))*param.g*tan(ui(2)) ...
      xf(1) uf(1)*cos(xf(3)) -sin(xf(3))*param.g*tan(uf(2)) ...
      xf(2) uf(1)*sin(xf(3))  cos(xf(3))*param.g*tan(uf(2)) ]';

a = M \ b;

z1_poly   = @(t)(a(1)+a(2)*t+a(3)*t.^2+a(4)*t.^3+a(5)*t.^4+a(6)*t.^5);
z2_poly   = @(t)(a(7)+a(8)*t+a(9)*t.^2+a(10)*t.^3+a(11)*t.^4+a(12)*t.^5);
zd1_poly  = @(t)(a(2)+2*a(3)*t+3*a(4)*t.^2+4*a(5)*t.^3+5*a(6)*t.^4);
zd2_poly  = @(t)(a(8)+2*a(9)*t+3*a(10)*t.^2+4*a(11)*t.^3+5*a(12)*t.^4);
zdd1_poly = @(t)(2*a(3)+2*3*a(4)*t+3*4*a(5)*t.^2+4*5*a(6)*t.^3);
zdd2_poly = @(t)(2*a(9)+2*3*a(10)*t+3*4*a(11)*t.^2+4*5*a(12)*t.^3);

X_poly = @(t)([z1_poly(t); z2_poly(t); atan2(zd2_poly(t), zd1_poly(t))]);
U_poly = @(t)([sqrt((zd1_poly(t).^2)+(zd2_poly(t).^2)); ...
               atan2((zdd2_poly(t).*zd1_poly(t)-zd2_poly(t).*zdd1_poly(t)), ...
                     (param.g.*(sqrt((zd1_poly(t).^2)+(zd2_poly(t).^2)))))]);

% =========================================================================
% PART B: BEZIER CURVE (Exercise 2.3)
% =========================================================================
n = 5;

% Calculation of control points P from boundary conditions:
z0   = [xi(1); xi(2)];
zd0  = [ui(1)*cos(xi(3)); ui(1)*sin(xi(3))];
zdd0 = [-sin(xi(3))*param.g*tan(ui(2)); cos(xi(3))*param.g*tan(ui(2))];

zf   = [xf(1); xf(2)];
zdf  = [uf(1)*cos(xf(3)); uf(1)*sin(xf(3))];
zddf = [-sin(xf(3))*param.g*tan(uf(2)); cos(xf(3))*param.g*tan(uf(2))];

P = zeros(2, n+1);
P(:, 1) = z0;
P(:, 2) = P(:, 1) + (T / n) * zd0;
P(:, 3) = 2*P(:, 2) - P(:, 1) + (T^2 / (n*(n-1))) * zdd0;

P(:, 6) = zf;
P(:, 5) = P(:, 6) - (T / n) * zdf;
P(:, 4) = 2*P(:, 5) - P(:, 6) + (T^2 / (n*(n-1))) * zddf;

% Definition of planar output and derivatives using their generic functions.
z1   = @(t) bezier(t/T, P(1,:));
z2   = @(t) bezier(t/T, P(2,:));

zd1  = @(t) (1/T) * bezier_dot(t/T, P(1,:));
zd2  = @(t) (1/T) * bezier_dot(t/T, P(2,:));

zdd1 = @(t) (1/T^2) * bezier_ddot(t/T, P(1,:));
zdd2 = @(t) (1/T^2) * bezier_ddot(t/T, P(2,:));

X = @(t)([z1(t); z2(t); atan2(zd2(t), zd1(t))]);
U = @(t)([sqrt((zd1(t).^2)+(zd2(t).^2)); ...
          atan2((zdd2(t).*zd1(t)-zd2(t).*zdd1(t)), ...
                (param.g.*(sqrt((zd1(t).^2)+(zd2(t).^2)))))]);

% =========================================================================
% EVALUATION AND COMPARISON PLOTS
% =========================================================================
tt = 0 : param.h : T;

xi_ref_poly = X_poly(tt);
u_ref_poly  = U_poly(tt);

xi_ref = X(tt);
u_ref  = U(tt);

% Figure 1: Control points and convex Bezier polygon
figure(1);
hold on; grid on; axis equal;
plot(P(1,:), P(2,:), 'or--', 'LineWidth', 1.5, 'MarkerSize', 8, 'MarkerFaceColor', 'r');
plot(xi_ref(1,:), xi_ref(2,:), 'b-', 'LineWidth', 2);
title('Bézier Curve and Control Points P_i');
xlabel('x [m]'); ylabel('y [m]');
legend('Control Points P_i', 'Bézier Trajectory', 'Location', 'NorthWest');

% Figura 2: Comparacao direta (Bezier vs Polinomio Canonico)
figure(2);
subplot(2,2,1);
plot(xi_ref_poly(1,:), xi_ref_poly(2,:), 'k-', 'LineWidth', 2.5); hold on;
plot(xi_ref(1,:), xi_ref(2,:), 'b--', 'LineWidth', 2);
grid on; xlabel('x [m]'); ylabel('y [m]'); title('Trajectory (x, y)');
legend('Polynomial (Ex 2.1)', 'Bézier (Ex 2.3)', 'Location', 'NorthWest');

subplot(2,2,2);
plot(tt, xi_ref_poly(3,:), 'k-', 'LineWidth', 2.5); hold on;
plot(tt, xi_ref(3,:), 'r--', 'LineWidth', 2);
grid on; xlabel('t [s]'); ylabel('\Psi [rad]'); title('Heading (Yaw)');

subplot(2,2,3);
plot(tt, u_ref_poly(1,:), 'k-', 'LineWidth', 2.5); hold on;
plot(tt, u_ref(1,:), 'g--', 'LineWidth', 2);
grid on; xlabel('t [s]'); ylabel('V_a [m/s]'); title('Velocity V_a');

subplot(2,2,4);
plot(tt, u_ref_poly(2,:), 'k-', 'LineWidth', 2.5); hold on;
plot(tt, u_ref(2,:), 'm--', 'LineWidth', 2);
grid on; xlabel('t [s]'); ylabel('\Phi [rad]'); title('Roll Angle \Phi');

% =========================================================================
% FUNCOES LOCAIS (Corrigidas com * no lugar de -)
% =========================================================================
function z = bezier(t, P)
    n = length(P) - 1;
    z = 0;
    for i = 0:n
        bernstein = prod(1:n)/(prod(1:i)*prod(1:n-i)) ...
                    * (1-t).^(n-i) .* t.^i;
        z = z + P(i+1) .* bernstein;
    end
end

function zd = bezier_dot(t, P)
    n = length(P) - 1;
    zd = 0;
    for i = 0:n-1
        bernstein = prod(1:n-1)/(prod(1:i)*prod(1:n-1-i)) ...
                    * (1-t).^(n-1-i) .* t.^i;
        zd = zd + n * (P(i+2) - P(i+1)) .* bernstein;
    end
end

function zdd = bezier_ddot(t, P)
    n = length(P) - 1;
    zdd = 0;
    for i = 0:n-2
        bernstein = prod(1:n-2)/(prod(1:i)*prod(1:n-2-i)) ...
                    * (1-t).^(n-2-i) .* t.^i;
        zdd = zdd + n * (n-1) * (P(i+3) - 2*P(i+2) + P(i+1)) .* bernstein;
    end
end