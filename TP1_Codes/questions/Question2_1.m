clear;
clc;
close all;

%% ============================================================
%  EXERCISE 2.1
%  UAV trajectory generation using differential flatness
%  Polynomial parameterization + one exact waypoint
%
%  z1(t) = x(t)
%  z2(t) = y(t)
%
%  With one waypoint:
%       7 conditions per coordinate
%       -> polynomial degree 6
% =============================================================


%% ============================================================
%  INITIALIZE SYSTEM PARAMETERS
% =============================================================

param.g = 9.8;        % gravity [m/s^2]
param.h = 0.05;       % sampling period [s]
param.dblAlt = 150;   % altitude (not used in this 2D model)


%% ============================================================
%  INITIALIZE REFERENCES
% =============================================================

t = [];
xi_ref = [];
u_ref  = [];


%% ============================================================
%  INITIAL AND FINAL CONDITIONS
%
%  xi = [x ; y ; Psi]
%  ui = [Va ; Phi]
%
% =============================================================

xi = [ ...
    0;          % x_i [m]
    0;          % y_i [m]
    0           % Psi_i [rad]
];

ui = [ ...
    20;         % Va_i [m/s]
    0           % Phi_i [rad]
];

xf = [ ...
    300;        % x_f [m]
    100;        % y_f [m]
    pi/6        % Psi_f [rad]
];

uf = [ ...
    20;         % Va_f [m/s]
    0           % Phi_f [rad]
];

T = 15;         % final time [s]


%% ============================================================
%  WAYPOINT
% =============================================================

tw = T/2;

wp = [ ...
    150;        % x waypoint [m]
    35          % y waypoint [m]
];


%% ============================================================
%  POLYNOMIAL PARAMETERIZATION - DEGREE 6

M = [ ...

1 0 0 0 0 0 0      0 0 0 0 0 0 0; ...    % z1(0)

0 1 0 0 0 0 0      0 0 0 0 0 0 0; ...    % zd1(0)

0 0 2 0 0 0 0      0 0 0 0 0 0 0; ...    % zdd1(0)

0 0 0 0 0 0 0      1 0 0 0 0 0 0; ...    % z2(0)

0 0 0 0 0 0 0      0 1 0 0 0 0 0; ...    % zd2(0)

0 0 0 0 0 0 0      0 0 2 0 0 0 0; ...    % zdd2(0)

1 T T^2 T^3 T^4 T^5 T^6 ...
0 0 0 0 0 0 0; ...

0 1 2*T 3*T^2 4*T^3 5*T^4 6*T^5 ...
0 0 0 0 0 0 0; ...

0 0 2 6*T 12*T^2 20*T^3 30*T^4 ...
0 0 0 0 0 0 0; ...

0 0 0 0 0 0 0 ...
1 T T^2 T^3 T^4 T^5 T^6; ...

0 0 0 0 0 0 0 ...
0 1 2*T 3*T^2 4*T^3 5*T^4 6*T^5; ...

0 0 0 0 0 0 0 ...
0 0 2 6*T 12*T^2 20*T^3 30*T^4; ...


% ------------------------------------------------------------
% WAYPOINT z1(tw) = wp(1)
% ------------------------------------------------------------

1 tw tw^2 tw^3 tw^4 tw^5 tw^6 ...
0 0 0 0 0 0 0; ...


% ------------------------------------------------------------
% WAYPOINT z2(tw) = wp(2)
% ------------------------------------------------------------

0 0 0 0 0 0 0 ...
1 tw tw^2 tw^3 tw^4 tw^5 tw^6
];


%% ============================================================
%  VECTOR b
%
% Boundary conditions written in terms of:
%
% z, zd, zdd
%
% =============================================================

b = [ ...

% z1 at initial time
xi(1); ...

% zd1(0) = Va_i*cos(Psi_i)
ui(1)*cos(xi(3)); ...

% zdd1(0)
-sin(xi(3))*param.g*tan(ui(2)); ...


% z2 at initial time
xi(2); ...

% zd2(0) = Va_i*sin(Psi_i)
ui(1)*sin(xi(3)); ...

% zdd2(0)
cos(xi(3))*param.g*tan(ui(2)); ...


% z1 at final time
xf(1); ...

% zd1(T)
uf(1)*cos(xf(3)); ...

% zdd1(T)
-sin(xf(3))*param.g*tan(uf(2)); ...

% z2 at final time
xf(2); ...

% zd2(T)
uf(1)*sin(xf(3)); ...

% zdd2(T)
cos(xf(3))*param.g*tan(uf(2)); ...

% waypoint x
wp(1); ...

% waypoint y
wp(2)
];


%% ============================================================
%  SOLVE M*a = b
% =============================================================

if rank(M) < 14
    error('Matrix M is singular. Check T and waypoint time tw.');
end

a = M\b;


%% ============================================================
%  DEFINE FLAT OUTPUT z1(t), z2(t)
%
%  DEGREE 6
% =============================================================

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
%  FIRST DERIVATIVES
% =============================================================

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
%  SECOND DERIVATIVES
% =============================================================

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
%  RECONSTRUCT STATE REFERENCE
%
%  xi_ref = [x ; y ; Psi]
% =============================================================

X = @(t) ([
    feval(z1,t);
    feval(z2,t);
    atan2( ...
        feval(zd2,t), ...
        feval(zd1,t) ...
    )
]);


%% ============================================================
%  RECONSTRUCT INPUT REFERENCE
%
%  u_ref = [Va ; Phi]
% =============================================================

U = @(t) [ ...
    sqrt( ...
    feval(zd1,t).^2 + feval(zd2,t).^2 ...
    ); ...
    atan2( ...
    feval(zdd2,t).*feval(zd1,t) - ...
    feval(zd2,t).*feval(zdd1,t), ...
    param.g .* sqrt( ...
    feval(zd1,t).^2 + feval(zd2,t).^2 ...
    ) ...
    ) ...
    ];


%% ============================================================
%  AIRSPEED ACCELERATION
%
% Va = sqrt(zd1^2 + zd2^2)
%
% dVa/dt =
%
% (zd1*zdd1 + zd2*zdd2) / Va
%
% =============================================================

Vad = @(t) ( ...
      feval(zd1,t).*feval(zdd1,t) ...
    + feval(zd2,t).*feval(zdd2,t) ...
    ) ./ ...
    sqrt( ...
        feval(zd1,t).^2 + ...
        feval(zd2,t).^2 ...
    );


%% ============================================================
% TOTAL PLANAR ACCELERATION MAGNITUDE
%
% ||a|| = sqrt(zdd1^2 + zdd2^2)
% =============================================================

Atotal = @(t) sqrt( ...
    feval(zdd1,t).^2 + ...
    feval(zdd2,t).^2 ...
);


%% ============================================================
%  COMPUTE STATE AND INPUT REFERENCES
% =============================================================

tt = 0:param.h:T;

xi_ref = [xi_ref feval(X,tt)];

u_ref = [u_ref feval(U,tt)];

acc_ref = feval(Vad,tt);

atotal_ref = feval(Atotal,tt);


%% ============================================================
%  CHECK WAYPOINT
% =============================================================

z_waypoint = [
    feval(z1,tw);
    feval(z2,tw)
];

disp('------------------------------------')
disp('Required waypoint:')
disp(wp)

disp('Computed trajectory at tw:')
disp(z_waypoint)

disp('Waypoint error:')
disp(norm(z_waypoint - wp))


%% ============================================================
%  CHECK INITIAL AND FINAL CONDITIONS
% =============================================================

disp('------------------------------------')
disp('Initial reference state:')
disp(xi_ref(:,1))

disp('Desired initial state:')
disp(xi)

disp('Final reference state:')
disp(xi_ref(:,end))

disp('Desired final state:')
disp(xf)


%% ============================================================
%  ANIMATION XY TRAJECTORY
% =============================================================

figure;
clf;

hold on;
grid on;
axis equal;

margin = 20;

xlim([min(xi_ref(1,:))-margin, max(xi_ref(1,:))+margin]);
ylim([min(xi_ref(2,:))-margin, max(xi_ref(2,:))+margin]);

plot( ...
    xi(1), xi(2), ...
    'o', ...
    'MarkerSize', 8, ...
    'LineWidth', 2, ...
    'DisplayName', 'Initial point' ...
    );

plot( ...
    wp(1), wp(2), ...
    's', ...
    'MarkerSize', 9, ...
    'LineWidth', 2, ...
    'DisplayName', 'Waypoint' ...
    );

plot( ...
    xf(1), xf(2), ...
    'x', ...
    'MarkerSize', 10, ...
    'LineWidth', 2, ...
    'DisplayName', 'Final point' ...
    );

hTrail = animatedline( ...
    'LineWidth', 2, ...
    'DisplayName', 'Trajectory' ...
    );

%% UAV
hUAV = plot( ...
    xi_ref(1,1), ...
    xi_ref(2,1), ...
    'o', ...
    'MarkerSize', 12, ...
    'MarkerFaceColor', 'auto', ...
    'LineWidth', 2, ...
    'DisplayName', 'UAV' ...
    );

xlabel('x [m]');
ylabel('y [m]');

title('UAV polynomial reference trajectory');

legend('Location','best');

%% ============================================================
%  ANIMATION
% =============================================================

for k = 1:size(xi_ref,2)

    addpoints( ...
        hTrail, ...
        xi_ref(1,k), ...
        xi_ref(2,k) ...
        );

    set( ...
        hUAV, ...
        'XData', xi_ref(1,k), ...
        'YData', xi_ref(2,k) ...
        );

    drawnow;

    pause(0.03);

end


%% ============================================================
%  PLOT AIRSPEED
% =============================================================

figure;

plot(tt, u_ref(1,:), 'LineWidth', 2);

hold on;

yline(18, '--');
yline(25, '--');

grid on;

xlabel('Time [s]');
ylabel('V_a [m/s]');

title('Airspeed');

legend( ...
    'V_a(t)', ...
    'Minimum', ...
    'Maximum' ...
);


%% ============================================================
%  PLOT BANK ANGLE
% =============================================================

figure;

plot(tt, u_ref(2,:), 'LineWidth', 2);

hold on;

yline(-0.43, '--');
yline(0.43, '--');

grid on;

xlabel('Time [s]');
ylabel('\Phi [rad]');

title('Bank angle');

legend( ...
    '\Phi(t)', ...
    'Minimum', ...
    'Maximum' ...
);


%% ============================================================
%  PLOT AIRSPEED ACCELERATION
%
% This corresponds to dVa/dt
% =============================================================

figure;

plot(tt, acc_ref, 'LineWidth', 2);

hold on;

yline(0.2, '--');
yline(-0.2, '--');

grid on;

xlabel('Time [s]');
ylabel('dV_a/dt [m/s^2]');

title('Airspeed acceleration');

legend( ...
    'dV_a/dt', ...
    '+0.2 m/s^2', ...
    '-0.2 m/s^2' ...
);


%% ============================================================
%  OPTIONAL PLOT: TOTAL PLANAR ACCELERATION
% =============================================================

figure;

plot(tt, atotal_ref, 'LineWidth', 2);

grid on;

xlabel('Time [s]');
ylabel('||a|| [m/s^2]');

title('Total planar acceleration');


%% ============================================================
%  NUMERICAL INFORMATION
% =============================================================

fprintf('\n=========== RESULTS ===========\n');

fprintf('Waypoint error = %.8e m\n', ...
    norm(z_waypoint - wp));

fprintf('\nAirspeed:\n');
fprintf('min Va = %.4f m/s\n', min(u_ref(1,:)));
fprintf('max Va = %.4f m/s\n', max(u_ref(1,:)));

fprintf('\nBank angle:\n');
fprintf('min Phi = %.4f rad\n', min(u_ref(2,:)));
fprintf('max Phi = %.4f rad\n', max(u_ref(2,:)));

fprintf('\nAirspeed acceleration:\n');
fprintf('min dVa/dt = %.4f m/s^2\n', min(acc_ref));
fprintf('max dVa/dt = %.4f m/s^2\n', max(acc_ref));