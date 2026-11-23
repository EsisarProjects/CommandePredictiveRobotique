 clear;
clc;
close all;

%% ============================================================
% EXERCISE 2.3
%
% UAV trajectory generation using a Bezier parameterization
%
% Flat output:
%
%       z = [x ; y]
%
% Degree 6 Bezier curve
% + exact waypoint
% + velocity
% + acceleration
% ============================================================


%% ============================================================
% SYSTEM PARAMETERS
% ============================================================

param.g = 9.8;
param.h = 0.05;
param.dblAlt = 150;


%% ============================================================
% INITIAL AND FINAL CONDITIONS
%
% xi = [x ; y ; Psi]
% u  = [Va ; Phi]
% ============================================================

xi = [ ...
    0;
    0;
    0
];

ui = [ ...
    20;
    0
];

xf = [ ...
    300;
    100;
    pi/6
];

uf = [ ...
    20;
    0
];

T = 15;


%% ============================================================
% WAYPOINT
%
% The trajectory must pass exactly through:
%
%       z(tw) = wp
%
% ============================================================

tw = T/2;

wp = [ ...
    150;
    35
];


%% ============================================================
% BEZIER DEGREE
% ============================================================

n = 6;


%% ============================================================
% INITIAL AND FINAL FLAT OUTPUT
% ============================================================

zi = [ ...
    xi(1);
    xi(2)
];

zf = [ ...
    xf(1);
    xf(2)
];


%% ============================================================
% INITIAL AND FINAL VELOCITY
%
% zd =
%
% [ Va*cos(Psi) ]
% [ Va*sin(Psi) ]
% ============================================================

vi = [ ...
    ui(1)*cos(xi(3));
    ui(1)*sin(xi(3))
];

vf = [ ...
    uf(1)*cos(xf(3));
    uf(1)*sin(xf(3))
];


%% ============================================================
% INITIAL AND FINAL ACCELERATION
%
% Assuming dVa/dt = 0 at the boundaries:
%
% zdd1 = -g*sin(Psi)*tan(Phi)
% zdd2 =  g*cos(Psi)*tan(Phi)
% ============================================================

ai = [ ...
    -param.g*sin(xi(3))*tan(ui(2));
     param.g*cos(xi(3))*tan(ui(2))
];

af = [ ...
    -param.g*sin(xf(3))*tan(uf(2));
     param.g*cos(xf(3))*tan(uf(2))
];


%% ============================================================
% CONTROL POINTS FROM INITIAL CONDITIONS
% ============================================================

P0 = zi;

P1 = P0 + (T/n)*vi;

P2 = (T^2/(n*(n-1)))*ai ...
     + 2*P1 ...
     - P0;


%% ============================================================
% CONTROL POINTS FROM FINAL CONDITIONS
% ============================================================

P6 = zf;

P5 = P6 - (T/n)*vf;

P4 = (T^2/(n*(n-1)))*af ...
     - P6 ...
     + 2*P5;


%% ============================================================
% BERNSTEIN BASIS
%
% B_i,n(tau) =
%
% nchoosek(n,i)*(1-tau)^(n-i)*tau^i
%
% tau belongs to [0,1]
% ============================================================

B = @(i,n,tau) ...
    nchoosek(n,i) .* ...
    (1-tau).^(n-i) .* ...
    tau.^i;


%% ============================================================
% COMPUTE P3 USING THE EXACT WAYPOINT
%
% waypoint condition:
%
% wp = sum(Pi * B_i,6(tau_w))
%
% Only P3 is unknown.
% ============================================================

tau_w = tw/T;

B0w = B(0,n,tau_w);
B1w = B(1,n,tau_w);
B2w = B(2,n,tau_w);
B3w = B(3,n,tau_w);
B4w = B(4,n,tau_w);
B5w = B(5,n,tau_w);
B6w = B(6,n,tau_w);


P3 = ( ...
      wp ...
    - P0*B0w ...
    - P1*B1w ...
    - P2*B2w ...
    - P4*B4w ...
    - P5*B5w ...
    - P6*B6w ...
    ) / B3w;


%% ============================================================
% CONTROL POINT MATRIX
%
% Each column is one control point:
%
% P = [P0 P1 P2 P3 P4 P5 P6]
%
% Dimension: 2 x 7
% ============================================================

P = [ ...
    P0 P1 P2 P3 P4 P5 P6 ...
];

disp('Control points:')
disp(P)

disp('Size of P:')
disp(size(P))


%% ============================================================
% BEZIER CURVE z(t)
% ============================================================

z = @(t) ...
      P(:,1)*B(0,6,t/T) ...
    + P(:,2)*B(1,6,t/T) ...
    + P(:,3)*B(2,6,t/T) ...
    + P(:,4)*B(3,6,t/T) ...
    + P(:,5)*B(4,6,t/T) ...
    + P(:,6)*B(5,6,t/T) ...
    + P(:,7)*B(6,6,t/T);


%% ============================================================
% FIRST DERIVATIVE
%
% zd(t) =
%
% n/T *
% sum((P_i+1 - P_i) B_i,n-1)
%
% Here:
%
% n = 6
% ============================================================

zd = @(t) (6/T) * ( ...
    (P(:,2)-P(:,1))*B(0,5,t/T) ...
    + (P(:,3)-P(:,2))*B(1,5,t/T) ...
    + (P(:,4)-P(:,3))*B(2,5,t/T) ...
    + (P(:,5)-P(:,4))*B(3,5,t/T) ...
    + (P(:,6)-P(:,5))*B(4,5,t/T) ...
    + (P(:,7)-P(:,6))*B(5,5,t/T) ...
    );


%% ============================================================
% SECOND DERIVATIVE
%
% zdd(t) =
%
% n(n-1)/T^2 *
%
% sum(
% P_i+2 - 2*P_i+1 + P_i
% )
%
% n(n-1) = 6*5 = 30
% ============================================================

zdd = @(t) (30/T^2) * ( ...
    (P(:,3)-2*P(:,2)+P(:,1))*B(0,4,t/T) ...
    + (P(:,4)-2*P(:,3)+P(:,2))*B(1,4,t/T) ...
    + (P(:,5)-2*P(:,4)+P(:,3))*B(2,4,t/T) ...
    + (P(:,6)-2*P(:,5)+P(:,4))*B(3,4,t/T) ...
    + (P(:,7)-2*P(:,6)+P(:,5))*B(4,4,t/T) ...
    );


%% ============================================================
% TIME VECTOR
% ============================================================

tt = 0:param.h:T;


%% ============================================================
% EVALUATE z, zd AND zdd
% ============================================================

Z   = z(tt);
ZD  = zd(tt);
ZDD = zdd(tt);


%% ============================================================
% STATE RECONSTRUCTION
%
% Psi = atan2(zd2,zd1)
% ============================================================

Psi = atan2( ...
    ZD(2,:), ...
    ZD(1,:) ...
);

xi_ref = [ ...
    Z(1,:);
    Z(2,:);
    Psi
];


%% ============================================================
% AIRSPEED
% ============================================================

Va = sqrt( ...
    ZD(1,:).^2 ...
    + ZD(2,:).^2 ...
);


%% ============================================================
% BANK ANGLE
% ============================================================

Phi = atan2( ...
    ZDD(2,:).*ZD(1,:) ...
    - ZD(2,:).*ZDD(1,:), ...
    param.g .* Va ...
);


%% ============================================================
% INPUT REFERENCE
%
% u = [Va ; Phi]
% ============================================================

u_ref = [ ...
    Va;
    Phi
];


%% ============================================================
% AIRSPEED ACCELERATION
% ============================================================

Va_dot = ( ...
    ZD(1,:).*ZDD(1,:) ...
    + ZD(2,:).*ZDD(2,:) ...
    ) ./ Va;


%% ============================================================
% TOTAL PLANAR ACCELERATION
%
% ||a|| =
%
% sqrt(zdd1^2 + zdd2^2)
% ============================================================

A_total = sqrt( ...
    ZDD(1,:).^2 ...
    + ZDD(2,:).^2 ...
);


%% ============================================================
% CHECK INITIAL CONDITIONS
% ============================================================

disp('========================================')
disp('Initial position')
disp(z(0))
disp('Expected')
disp(zi)

disp('Initial velocity')
disp(zd(0))
disp('Expected')
disp(vi)

disp('Initial acceleration')
disp(zdd(0))
disp('Expected')
disp(ai)


%% ============================================================
% CHECK FINAL CONDITIONS
% ============================================================

disp('========================================')
disp('Final position')
disp(z(T))
disp('Expected')
disp(zf)

disp('Final velocity')
disp(zd(T))
disp('Expected')
disp(vf)

disp('Final acceleration')
disp(zdd(T))
disp('Expected')
disp(af)


%% ============================================================
% CHECK WAYPOINT
% ============================================================

Zwp = z(tw);

disp('========================================')
disp('Required waypoint:')
disp(wp)

disp('Obtained waypoint:')
disp(Zwp)

disp('Waypoint error:')
disp(norm(Zwp-wp))


%% ============================================================
% ANIMATION - BEZIER TRAJECTORY
% ============================================================

figure;
clf;

hold on;
grid on;
axis equal;

%% Limites fixos do gráfico
margin = 20;

x_all = [Z(1,:), P(1,:), zi(1), wp(1), zf(1)];
y_all = [Z(2,:), P(2,:), zi(2), wp(2), zf(2)];

xlim([min(x_all)-margin, max(x_all)+margin]);
ylim([min(y_all)-margin, max(y_all)+margin]);

%% ============================================================
% CONTROL POLYGON
% ============================================================

plot( ...
    P(1,:), ...
    P(2,:), ...
    '--o', ...
    'LineWidth',1.5, ...
    'DisplayName','Control polygon' ...
);


%% Convex hull
K = convhull(P(1,:), P(2,:));

fill( ...
    P(1,K), ...
    P(2,K), ...
    [0.9 0.9 0.9], ...
    'FaceAlpha',0.25, ...
    'EdgeColor','k', ...
    'LineWidth',1.5, ...
    'DisplayName','Convex hull' ...
    );

%% ============================================================
% INITIAL POINT
% ============================================================

plot( ...
    zi(1), zi(2), ...
    'o', ...
    'MarkerSize',9, ...
    'LineWidth',2, ...
    'DisplayName','Initial point' ...
);

%% ============================================================
% WAYPOINT
% ============================================================

plot( ...
    wp(1), wp(2), ...
    's', ...
    'MarkerSize',10, ...
    'LineWidth',2, ...
    'DisplayName','Waypoint' ...
);

%% ============================================================
% FINAL POINT
% ============================================================

plot( ...
    zf(1), zf(2), ...
    'x', ...
    'MarkerSize',10, ...
    'LineWidth',2, ...
    'DisplayName','Final point' ...
);

%% ============================================================
% ANIMATED BEZIER TRAJECTORY
% ============================================================

hTrail = animatedline( ...
    'LineWidth',2, ...
    'DisplayName','Bezier trajectory' ...
);

%% UAV MARKER
hUAV = plot( ...
    Z(1,1), ...
    Z(2,1), ...
    'o', ...
    'MarkerSize',12, ...
    'MarkerFaceColor','auto', ...
    'LineWidth',2, ...
    'DisplayName','UAV' ...
);

xlabel('x [m]');
ylabel('y [m]');

title('UAV Bezier trajectory with exact waypoint');

legend('Location','best');

%% ============================================================
% ANIMATION LOOP
% ============================================================

for k = 1:size(Z,2)

    % Add current point to trajectory
    addpoints( ...
        hTrail, ...
        Z(1,k), ...
        Z(2,k) ...
    );

    % Move UAV
    set( ...
        hUAV, ...
        'XData', Z(1,k), ...
        'YData', Z(2,k) ...
    );

    drawnow;

    pause(0.03);

end
%% ============================================================
% PLOT AIRSPEED
% ============================================================

figure;

plot( ...
    tt,Va, ...
    'LineWidth',2 ...
);

grid on;

xlabel('Time [s]');
ylabel('V_a [m/s]');

title('Airspeed');


%% ============================================================
% PLOT HEADING
% ============================================================

figure;

plot( ...
    tt,Psi, ...
    'LineWidth',2 ...
);

grid on;

xlabel('Time [s]');
ylabel('\Psi [rad]');

title('Heading angle');


%% ============================================================
% PLOT BANK ANGLE
% ============================================================

figure;

plot( ...
    tt,Phi, ...
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
% This represents dVa/dt
% ============================================================

figure;

plot( ...
    tt,Va_dot, ...
    'LineWidth',2 ...
);

hold on;

yline(0.2,'--');
yline(-0.2,'--');

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
% PLOT TOTAL PLANAR ACCELERATION
% ============================================================

figure;

plot( ...
    tt,A_total, ...
    'LineWidth',2 ...
);

grid on;

xlabel('Time [s]');
ylabel('||a|| [m/s^2]');

title('Total planar acceleration');


%% ============================================================
% NUMERICAL RESULTS
% ============================================================

fprintf('\n======================================\n');
fprintf('          BEZIER RESULTS\n');
fprintf('======================================\n');

fprintf( ...
    'Waypoint error = %.10e m\n', ...
    norm(Zwp-wp) ...
);


fprintf('\nAirspeed:\n');

fprintf( ...
    'Va min = %.4f m/s\n', ...
    min(Va) ...
);

fprintf( ...
    'Va max = %.4f m/s\n', ...
    max(Va) ...
);


fprintf('\nBank angle:\n');

fprintf( ...
    'Phi min = %.4f rad\n', ...
    min(Phi) ...
);

fprintf( ...
    'Phi max = %.4f rad\n', ...
    max(Phi) ...
);


fprintf('\nAirspeed acceleration:\n');

fprintf( ...
    'dVa/dt min = %.4f m/s^2\n', ...
    min(Va_dot) ...
);

fprintf( ...
    'dVa/dt max = %.4f m/s^2\n', ...
    max(Va_dot) ...
);


fprintf('\nTotal acceleration:\n');

fprintf( ...
    'max ||a|| = %.4f m/s^2\n', ...
    max(A_total) ...
);