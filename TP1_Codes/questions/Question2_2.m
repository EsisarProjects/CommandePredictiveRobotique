clear;
clc;
close all;

%% ============================================================
% EXERCISE 2.2
% UAV trajectory generation with CONSTANT AIRSPEED
%
% Polynomial parameterization + one exact waypoint
%
% z1(t) = x(t)
% z2(t) = y(t)
%
% One waypoint -> degree 6 polynomial
% ============================================================


%% ============================================================
% SYSTEM PARAMETERS
% ============================================================

param.g = 9.8;          % gravity [m/s^2]
param.h = 0.05;         % sampling time [s]
param.dblAlt = 150;     % altitude [m]

Va_const = 20;          % CONSTANT airspeed [m/s]


%% ============================================================
% INITIALIZE REFERENCES
% ============================================================

t = [];

xi_ref = [];
u_ref  = [];


%% ============================================================
% INITIAL AND FINAL CONDITIONS
%
% xi = [x ; y ; Psi]
% u  = [Va ; Phi]
% ============================================================

xi = [ ...
    0;          % x_i
    0;          % y_i
    0           % Psi_i
];

ui = [ ...
    Va_const;   % constant Va
    0           % initial Phi
];

xf = [ ...
    300;        % x_f
    100;        % y_f
    pi/6        % Psi_f
];

uf = [ ...
    Va_const;   % constant Va
    0           % final Phi
];

T = 15;         % final time [s]


%% ============================================================
% WAYPOINT
%
% z(tw) = wp
% ============================================================

tw = T/2;

wp = [ ...
    150;        % waypoint x
    35          % waypoint y
];


%% ============================================================
% MATRIX M
%
% Polynomial degree 6:
%
% z1 = a1 + a2*t + ... + a7*t^6
% z2 = a8 + a9*t + ... + a14*t^6
%
% 14 equations / 14 unknowns
% ============================================================

M = [ ...

% ------------------------------------------------------------
% z1 at t = 0
% ------------------------------------------------------------

1 0 0 0 0 0 0     0 0 0 0 0 0 0; ...
0 1 0 0 0 0 0     0 0 0 0 0 0 0; ...
0 0 2 0 0 0 0     0 0 0 0 0 0 0; ...


% ------------------------------------------------------------
% z2 at t = 0
% ------------------------------------------------------------

0 0 0 0 0 0 0     1 0 0 0 0 0 0; ...
0 0 0 0 0 0 0     0 1 0 0 0 0 0; ...
0 0 0 0 0 0 0     0 0 2 0 0 0 0; ...


% ------------------------------------------------------------
% z1 at t = T
% ------------------------------------------------------------

1 T T^2 T^3 T^4 T^5 T^6 ...
0 0 0 0 0 0 0; ...

0 1 2*T 3*T^2 4*T^3 5*T^4 6*T^5 ...
0 0 0 0 0 0 0; ...

0 0 2 6*T 12*T^2 20*T^3 30*T^4 ...
0 0 0 0 0 0 0; ...


% ------------------------------------------------------------
% z2 at t = T
% ------------------------------------------------------------

0 0 0 0 0 0 0 ...
1 T T^2 T^3 T^4 T^5 T^6; ...

0 0 0 0 0 0 0 ...
0 1 2*T 3*T^2 4*T^3 5*T^4 6*T^5; ...

0 0 0 0 0 0 0 ...
0 0 2 6*T 12*T^2 20*T^3 30*T^4; ...


% ------------------------------------------------------------
% waypoint for z1
% ------------------------------------------------------------

1 tw tw^2 tw^3 tw^4 tw^5 tw^6 ...
0 0 0 0 0 0 0; ...


% ------------------------------------------------------------
% waypoint for z2
% ------------------------------------------------------------

0 0 0 0 0 0 0 ...
1 tw tw^2 tw^3 tw^4 tw^5 tw^6
];


%% ============================================================
% VECTOR b
%
% Since Va is constant:
%
% zd1 = Va*cos(Psi)
% zd2 = Va*sin(Psi)
%
% zdd1 = -g*sin(Psi)*tan(Phi)
% zdd2 =  g*cos(Psi)*tan(Phi)
% ============================================================

b = [ ...

% ------------------------------------------------------------
% Initial conditions - x
% ------------------------------------------------------------

xi(1); ...

Va_const*cos(xi(3)); ...

-param.g*sin(xi(3))*tan(ui(2)); ...


% ------------------------------------------------------------
% Initial conditions - y
% ------------------------------------------------------------

xi(2); ...

Va_const*sin(xi(3)); ...

param.g*cos(xi(3))*tan(ui(2)); ...


% ------------------------------------------------------------
% Final conditions - x
% ------------------------------------------------------------

xf(1); ...

Va_const*cos(xf(3)); ...

-param.g*sin(xf(3))*tan(uf(2)); ...


% ------------------------------------------------------------
% Final conditions - y
% ------------------------------------------------------------

xf(2); ...

Va_const*sin(xf(3)); ...

param.g*cos(xf(3))*tan(uf(2)); ...


% ------------------------------------------------------------
% waypoint
% ------------------------------------------------------------

wp(1); ...
wp(2)
];


%% ============================================================
% SOLVE M*a = b
% ============================================================

if rank(M) < 14
    error('Matrix M is singular.');
end

a = M\b;


%% ============================================================
% FLAT OUTPUT z1(t), z2(t)
% ============================================================

z1 = @(t) ( ...
      a(1) ...
    + a(2).*t ...
    + a(3).*t.^2 ...
    + a(4).*t.^3 ...
    + a(5).*t.^4 ...
    + a(6).*t.^5 ...
    + a(7).*t.^6 ...
);

z2 = @(t) ( ...
      a(8) ...
    + a(9).*t ...
    + a(10).*t.^2 ...
    + a(11).*t.^3 ...
    + a(12).*t.^4 ...
    + a(13).*t.^5 ...
    + a(14).*t.^6 ...
);


%% ============================================================
% FIRST DERIVATIVES
% ============================================================

zd1 = @(t) ( ...
      a(2) ...
    + 2*a(3).*t ...
    + 3*a(4).*t.^2 ...
    + 4*a(5).*t.^3 ...
    + 5*a(6).*t.^4 ...
    + 6*a(7).*t.^5 ...
);

zd2 = @(t) ( ...
      a(9) ...
    + 2*a(10).*t ...
    + 3*a(11).*t.^2 ...
    + 4*a(12).*t.^3 ...
    + 5*a(13).*t.^4 ...
    + 6*a(14).*t.^5 ...
);


%% ============================================================
% SECOND DERIVATIVES
% ============================================================

zdd1 = @(t) ( ...
      2*a(3) ...
    + 6*a(4).*t ...
    + 12*a(5).*t.^2 ...
    + 20*a(6).*t.^3 ...
    + 30*a(7).*t.^4 ...
);

zdd2 = @(t) ( ...
      2*a(10) ...
    + 6*a(11).*t ...
    + 12*a(12).*t.^2 ...
    + 20*a(13).*t.^3 ...
    + 30*a(14).*t.^4 ...
);


%% ============================================================
% REFERENCE STATE
%
% xi_ref = [x ; y ; Psi]
%
% Psi = atan2(zd2, zd1)
% ============================================================

X = @(t) [ ...
    z1(t); ...
    z2(t); ...
    atan2(zd2(t),zd1(t)) ...
];


%% ============================================================
% REFERENCE INPUT
%
% CONSTANT SPEED:
%
% Va(t) = Va_const
%
% Phi =
%
% atan2(
%      zdd2*zd1 - zd2*zdd1 ,
%      g*Va_const
% )
%
% ============================================================

U = @(t) [ ...

    Va_const .* ones(size(t)); ...

    atan2( ...
        zdd2(t).*zd1(t) ...
        - zd2(t).*zdd1(t), ...
        param.g .* Va_const .* ones(size(t)) ...
    ) ...
];


%% ============================================================
% TIME VECTOR
% ============================================================

tt = 0:param.h:T;


%% ============================================================
% COMPUTE STATE AND INPUT REFERENCES
% ============================================================

xi_ref = X(tt);

u_ref = U(tt);


%% ============================================================
% CONSTANT AIRSPEED ACCELERATION
%
% Since Va is constant:
%
% dVa/dt = 0
% ============================================================

acc_ref = zeros(size(tt));


%% ============================================================
% ACTUAL SPEED GENERATED BY THE POLYNOMIAL
%
% This is used only to check whether:
%
% sqrt(zd1^2 + zd2^2) = Va_const
% ============================================================

Va_geom = sqrt( ...
    zd1(tt).^2 + ...
    zd2(tt).^2 ...
);


%% ============================================================
% TOTAL PLANAR ACCELERATION
%
% This may be non-zero even when Va is constant
% because the direction of velocity may change.
% ============================================================

Atotal = @(t) sqrt( ...
    zdd1(t).^2 + ...
    zdd2(t).^2 ...
);

atotal_ref = Atotal(tt);


%% ============================================================
% CHECK WAYPOINT
% ============================================================

z_wp = [ ...
    z1(tw); ...
    z2(tw) ...
];

disp('====================================')
disp('Required waypoint:')
disp(wp)

disp('Computed waypoint:')
disp(z_wp)

disp('Waypoint error:')
disp(norm(z_wp-wp))


%% ============================================================
% CHECK INITIAL AND FINAL STATES
% ============================================================

disp('====================================')

disp('Initial reference state:')
disp(xi_ref(:,1))

disp('Desired initial state:')
disp(xi)

disp('Final reference state:')
disp(xi_ref(:,end))

disp('Desired final state:')
disp(xf)


%% ============================================================
% PLOT XY TRAJECTORY
% ============================================================

figure;

plot( ...
    xi_ref(1,:), ...
    xi_ref(2,:), ...
    'LineWidth',2 ...
);

hold on;

plot( ...
    xi(1),xi(2), ...
    'o', ...
    'MarkerSize',8, ...
    'LineWidth',2 ...
);

plot( ...
    wp(1),wp(2), ...
    's', ...
    'MarkerSize',9, ...
    'LineWidth',2 ...
);

plot( ...
    xf(1),xf(2), ...
    'x', ...
    'MarkerSize',10, ...
    'LineWidth',2 ...
);

grid on;
axis equal;

xlabel('x [m]');
ylabel('y [m]');

title('UAV reference trajectory - Constant airspeed');

legend( ...
    'Reference trajectory', ...
    'Initial point', ...
    'Waypoint', ...
    'Final point', ...
    'Location','best' ...
);


%% ============================================================
% PLOT Va REFERENCE
%
% This is the Va imposed in U(t)
% ============================================================

figure;

plot( ...
    tt, ...
    u_ref(1,:), ...
    'LineWidth',2 ...
);

grid on;

xlabel('Time [s]');
ylabel('V_a [m/s]');

title('Constant reference airspeed');


%% ============================================================
% CHECK SPEED CONSISTENCY
%
% Compare:
%
% imposed Va_const
%
% with
%
% sqrt(zd1^2 + zd2^2)
% ============================================================

figure;

plot( ...
    tt, ...
    Va_geom, ...
    'LineWidth',2 ...
);

hold on;

yline( ...
    Va_const, ...
    '--', ...
    'LineWidth',1.5 ...
);

grid on;

xlabel('Time [s]');
ylabel('Speed [m/s]');

title('Constant-speed consistency check');

legend( ...
    'Speed generated by z(t)', ...
    'Desired constant V_a', ...
    'Location','best' ...
);


%% ============================================================
% PLOT BANK ANGLE
% ============================================================

figure;

plot( ...
    tt, ...
    u_ref(2,:), ...
    'LineWidth',2 ...
);

hold on;

yline(0.43,'--');
yline(-0.43,'--');

grid on;

xlabel('Time [s]');
ylabel('\Phi [rad]');

title('Bank angle');

legend( ...
    '\Phi(t)', ...
    '+0.43 rad', ...
    '-0.43 rad' ...
);


%% ============================================================
% PLOT AIRSPEED ACCELERATION
%
% dVa/dt = 0
% ============================================================

figure;

plot( ...
    tt, ...
    acc_ref, ...
    'LineWidth',2 ...
);

grid on;

xlabel('Time [s]');
ylabel('dV_a/dt [m/s^2]');

title('Airspeed acceleration');

ylim([-0.1 0.1]);


%% ============================================================
% PLOT TOTAL PLANAR ACCELERATION
%
% Unlike dVa/dt, this may not be zero.
% ============================================================

figure;

plot( ...
    tt, ...
    atotal_ref, ...
    'LineWidth',2 ...
);

grid on;

xlabel('Time [s]');
ylabel('||a|| [m/s^2]');

title('Total planar acceleration');


%% ============================================================
% NUMERICAL RESULTS
% ============================================================

fprintf('\n=========== RESULTS ===========\n');

fprintf( ...
    'Waypoint error = %.8e m\n', ...
    norm(z_wp-wp) ...
);

fprintf('\nReference airspeed:\n');

fprintf( ...
    'Va constant = %.4f m/s\n', ...
    Va_const ...
);

fprintf('\nGeometric speed generated by polynomial:\n');

fprintf( ...
    'min = %.4f m/s\n', ...
    min(Va_geom) ...
);

fprintf( ...
    'max = %.4f m/s\n', ...
    max(Va_geom) ...
);

fprintf( ...
    'max |Va_geom - Va_const| = %.4f m/s\n', ...
    max(abs(Va_geom - Va_const)) ...
);

fprintf('\nBank angle:\n');

fprintf( ...
    'min Phi = %.4f rad\n', ...
    min(u_ref(2,:)) ...
);

fprintf( ...
    'max Phi = %.4f rad\n', ...
    max(u_ref(2,:)) ...
);