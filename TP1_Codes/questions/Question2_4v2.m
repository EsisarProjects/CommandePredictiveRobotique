%% ============================================================
% MULTI-UAV TRAJECTORY PLANNING
%
% APPROACH 2 - CORRECTED WAYPOINT ASSOCIATION
%
% 8 UAVs - 2D Swapping Square
% Bezier curves - degree 6
%
% Same waypoint crossing time:
%       tw = T/2
%
% Different waypoint for each UAV:
%       waypoint obtained by rotating the initial radial
%       direction by +90 degrees around the square center
%
% Flat output:
%       z = [x ; y]
%
% UAV model:
%
%       x_dot   = Va*cos(Psi)
%       y_dot   = Va*sin(Psi)
%       Psi_dot = g*tan(Phi)/Va
%
% ============================================================

clear;
clc;
close all;


%% ============================================================
% 1. PARAMETERS
% ============================================================

g = 9.81;                   % Gravity [m/s^2]

L = 300;                    % Square side [m]

T = 25;                     % Total trajectory time [s]

N = 1500;                   % Number of samples

t = linspace(0,T,N);

tau = t/T;                  % Normalized time

nUAV = 8;

V0 = 20;                    % Initial/final airspeed [m/s]

d_safe = 15;                % Safety distance [m]


%% ============================================================
% WAYPOINT REGION
% ============================================================

rho = 40;                   % Radius around center [m]

C = [ ...
    L/2;
    L/2 ...
];


%% ============================================================
% 2. INITIAL POSITIONS
%
%
%           S7 ----- S6 ----- S5
%            |                |
%            |                |
%           S8       C        S4
%            |                |
%            |                |
%           S1 ----- S2 ----- S3
%
%
% ============================================================

S = [ ...
    0       L/2      L       L       L       L/2      0       0;
    0       0        0       L/2     L       L        L       L/2 ...
];


%% ============================================================
% 3. DESTINATIONS
%
% S1 <-> S5
% S2 <-> S6
% S3 <-> S7
% S4 <-> S8
%
% ============================================================

destination = [ ...
    5 6 7 8 1 2 3 4 ...
];


%% ============================================================
% 4. SAME WAYPOINT TIME
% ============================================================

tw = T/2;

tau_w = tw/T;


%% ============================================================
% 5. WAYPOINT GENERATION
%
% Each UAV waypoint is obtained by rotating the vector
% from the center to its initial position by +90 degrees.
%
% Therefore all UAVs move around the center in the same
% rotational direction.
%
% ============================================================

W = zeros(2,nUAV);

rotationAngle = pi/2;


for i = 1:nUAV

    %% Vector from center to UAV initial position

    radial = ...
        S(:,i) ...
        - C;


    %% Initial angular position around center

    theta_i = atan2( ...
        radial(2), ...
        radial(1) ...
    );


    %% Rotate +90 degrees

    alpha_i = ...
        theta_i ...
        + rotationAngle;


    %% Waypoint

    W(:,i) = ...
        C ...
        + rho * [ ...
            cos(alpha_i);
            sin(alpha_i) ...
        ];

end


%% ============================================================
% 6. PRINT WAYPOINTS
% ============================================================

fprintf('\n');
fprintf('============================================\n');
fprintf('WAYPOINTS\n');
fprintf('============================================\n');


for i = 1:nUAV

    fprintf( ...
        'UAV %d : (%8.3f , %8.3f)\n', ...
        i, ...
        W(1,i), ...
        W(2,i) ...
    );

end


%% ============================================================
% 7. INITIAL / FINAL CONDITIONS
% ============================================================

Va_i = V0*ones(1,nUAV);

Va_f = V0*ones(1,nUAV);


% Initial/final bank angle

Phi_i = zeros(1,nUAV);

Phi_f = zeros(1,nUAV);


% Heading

Psi_i = zeros(1,nUAV);

Psi_f = zeros(1,nUAV);


%% ============================================================
% INITIAL / FINAL HEADINGS
%
% Initial heading:
%       initial position -> waypoint
%
% Final heading:
%       waypoint -> final position
%
% ============================================================

for i = 1:nUAV

    zi = S(:,i);

    zf = S(:,destination(i));

    wi = W(:,i);


    %% Initial direction

    direction_i = ...
        wi ...
        - zi;


    Psi_i(i) = atan2( ...
        direction_i(2), ...
        direction_i(1) ...
    );


    %% Final direction

    direction_f = ...
        zf ...
        - wi;


    Psi_f(i) = atan2( ...
        direction_f(2), ...
        direction_f(1) ...
    );

end


%% ============================================================
% 8. BERNSTEIN BASIS
% ============================================================

B = @(i,n,s) ...
    nchoosek(n,i) .* ...
    (1-s).^(n-i) .* ...
    s.^i;


%% ============================================================
% 9. MEMORY ALLOCATION
% ============================================================

P_all = zeros(2,7,nUAV);


Z = zeros(2,N,nUAV);

ZD = zeros(2,N,nUAV);

ZDD = zeros(2,N,nUAV);


Psi = zeros(nUAV,N);

Va = zeros(nUAV,N);

Phi = zeros(nUAV,N);

Va_dot = zeros(nUAV,N);


%% ============================================================
% 10. GENERATE BEZIER TRAJECTORIES
% ============================================================

for uav = 1:nUAV


    %% --------------------------------------------------------
    % Initial / final positions
    % ---------------------------------------------------------

    zi = S(:,uav);

    zf = S(:,destination(uav));

    wi = W(:,uav);


    %% --------------------------------------------------------
    % Initial velocity
    % ---------------------------------------------------------

    vi = ...
        Va_i(uav) ...
        * [ ...
            cos(Psi_i(uav));
            sin(Psi_i(uav)) ...
        ];


    %% --------------------------------------------------------
    % Final velocity
    % ---------------------------------------------------------

    vf = ...
        Va_f(uav) ...
        * [ ...
            cos(Psi_f(uav));
            sin(Psi_f(uav)) ...
        ];


    %% --------------------------------------------------------
    % Initial acceleration
    %
    % Assume Va_dot = 0
    % ---------------------------------------------------------

    ai = [ ...
        -g*sin(Psi_i(uav))*tan(Phi_i(uav));
         g*cos(Psi_i(uav))*tan(Phi_i(uav)) ...
    ];


    %% --------------------------------------------------------
    % Final acceleration
    % ---------------------------------------------------------

    af = [ ...
        -g*sin(Psi_f(uav))*tan(Phi_f(uav));
         g*cos(Psi_f(uav))*tan(Phi_f(uav)) ...
    ];


    %% ========================================================
    % CONTROL POINTS
    % ========================================================

    P0 = zi;


    P1 = ...
        P0 ...
        + (T/6)*vi;


    P2 = ...
        (T^2/30)*ai ...
        + 2*P1 ...
        - P0;


    P6 = zf;


    P5 = ...
        P6 ...
        - (T/6)*vf;


    P4 = ...
        (T^2/30)*af ...
        - P6 ...
        + 2*P5;


    %% ========================================================
    % WAYPOINT CONDITION
    %
    % z(T/2) = wi
    %
    % Determines P3
    % ========================================================

    B0 = B(0,6,tau_w);
    B1 = B(1,6,tau_w);
    B2 = B(2,6,tau_w);
    B3 = B(3,6,tau_w);
    B4 = B(4,6,tau_w);
    B5 = B(5,6,tau_w);
    B6 = B(6,6,tau_w);


    P3 = ( ...
          wi ...
        - P0*B0 ...
        - P1*B1 ...
        - P2*B2 ...
        - P4*B4 ...
        - P5*B5 ...
        - P6*B6 ...
        ) / B3;


    %% Complete matrix

    P = [ ...
        P0 P1 P2 P3 P4 P5 P6 ...
    ];


    P_all(:,:,uav) = P;


    %% ========================================================
    % EVALUATE TRAJECTORY
    % ========================================================

    for k = 1:N

        s = tau(k);


        %% Position

        z = zeros(2,1);


        for j = 0:6

            z = ...
                z ...
                + P(:,j+1)*B(j,6,s);

        end


        Z(:,k,uav) = z;


        %% ----------------------------------------------------
        % First derivative
        % -----------------------------------------------------

        zd = zeros(2,1);


        for j = 0:5

            zd = ...
                zd ...
                + (6/T) ...
                * ( ...
                    P(:,j+2) ...
                    - P(:,j+1) ...
                  ) ...
                * B(j,5,s);

        end


        ZD(:,k,uav) = zd;


        %% ----------------------------------------------------
        % Second derivative
        % -----------------------------------------------------

        zdd = zeros(2,1);


        for j = 0:4

            zdd = ...
                zdd ...
                + (30/T^2) ...
                * ( ...
                    P(:,j+3) ...
                    - 2*P(:,j+2) ...
                    + P(:,j+1) ...
                  ) ...
                * B(j,4,s);

        end


        ZDD(:,k,uav) = zdd;

    end


    %% ========================================================
    % FLATNESS RECONSTRUCTION
    % ========================================================

    %% Heading

    Psi(uav,:) = atan2( ...
        ZD(2,:,uav), ...
        ZD(1,:,uav) ...
    );


    %% Airspeed

    Va(uav,:) = sqrt( ...
        ZD(1,:,uav).^2 ...
        + ...
        ZD(2,:,uav).^2 ...
    );


    %% Bank angle

    Phi(uav,:) = atan2( ...
        ZDD(2,:,uav).*ZD(1,:,uav) ...
        - ...
        ZD(2,:,uav).*ZDD(1,:,uav), ...
        g.*Va(uav,:) ...
    );


    %% Airspeed acceleration

    Va_dot(uav,:) = ( ...
        ZD(1,:,uav).*ZDD(1,:,uav) ...
        + ...
        ZD(2,:,uav).*ZDD(2,:,uav) ...
        ) ./ Va(uav,:);

end


%% ============================================================
% 11. EXACT WAYPOINT VERIFICATION
% ============================================================

fprintf('\n');
fprintf('============================================\n');
fprintf('EXACT WAYPOINT VERIFICATION\n');
fprintf('============================================\n');


for uav = 1:nUAV

    P = P_all(:,:,uav);

    z_wp = zeros(2,1);


    for j = 0:6

        z_wp = ...
            z_wp ...
            + P(:,j+1)*B(j,6,tau_w);

    end


    error_wp = norm( ...
        z_wp ...
        - W(:,uav) ...
    );


    fprintf( ...
        'UAV %d | waypoint = (%7.2f,%7.2f) | error = %.12e m\n', ...
        uav, ...
        W(1,uav), ...
        W(2,uav), ...
        error_wp ...
    );

end


%% ============================================================
% 12. WAYPOINT SEPARATION
% ============================================================

waypoint_min_distance = inf;

waypoint_pair = [0 0];


for i = 1:nUAV-1

    for j = i+1:nUAV

        dij = norm( ...
            W(:,i) ...
            - W(:,j) ...
        );


        if dij < waypoint_min_distance

            waypoint_min_distance = dij;

            waypoint_pair = [i j];

        end

    end

end


fprintf('\n');
fprintf('============================================\n');
fprintf('WAYPOINT SEPARATION\n');
fprintf('============================================\n');


fprintf( ...
    'Minimum waypoint separation = %.3f m\n', ...
    waypoint_min_distance ...
);


fprintf( ...
    'Waypoint pair = UAV %d / UAV %d\n', ...
    waypoint_pair(1), ...
    waypoint_pair(2) ...
);


%% ============================================================
% 13. COLLISION CHECK
% ============================================================

Dmin = inf(nUAV);

global_min_distance = inf;

critical_pair = [0 0];

critical_time = NaN;

critical_index = NaN;


for i = 1:nUAV-1

    for j = i+1:nUAV


        dx = ...
            Z(1,:,i) ...
            - Z(1,:,j);


        dy = ...
            Z(2,:,i) ...
            - Z(2,:,j);


        d = sqrt( ...
            dx.^2 ...
            + dy.^2 ...
        );


        [dmin_pair,idx] = min(d);


        Dmin(i,j) = dmin_pair;

        Dmin(j,i) = dmin_pair;


        if dmin_pair < global_min_distance

            global_min_distance = dmin_pair;

            critical_pair = [i j];

            critical_time = t(idx);

            critical_index = idx;

        end

    end

end


%% ============================================================
% 14. COLLISION RESULTS
% ============================================================

fprintf('\n');
fprintf('============================================\n');
fprintf('COLLISION CHECK\n');
fprintf('============================================\n');


fprintf( ...
    'Minimum distance = %.3f m\n', ...
    global_min_distance ...
);


fprintf( ...
    'Critical pair    = UAV %d / UAV %d\n', ...
    critical_pair(1), ...
    critical_pair(2) ...
);


fprintf( ...
    'Critical time    = %.3f s\n', ...
    critical_time ...
);


if global_min_distance >= d_safe

    fprintf('\nSAFE TRAJECTORIES\n');

    fprintf( ...
        'd_min = %.3f m >= d_safe = %.3f m\n', ...
        global_min_distance, ...
        d_safe ...
    );

else

    fprintf('\nWARNING: SAFETY DISTANCE VIOLATED\n');

    fprintf( ...
        'd_min = %.3f m < d_safe = %.3f m\n', ...
        global_min_distance, ...
        d_safe ...
    );

end


%% ============================================================
% 15. PAIRWISE MINIMUM DISTANCES
% ============================================================

fprintf('\n');
fprintf('============================================\n');
fprintf('PAIRWISE MINIMUM DISTANCES\n');
fprintf('============================================\n');


for i = 1:nUAV-1

    for j = i+1:nUAV


        if Dmin(i,j) >= d_safe

            status = 'SAFE';

        else

            status = 'WARNING';

        end


        fprintf( ...
            'UAV %d - UAV %d : %8.3f m   %s\n', ...
            i, ...
            j, ...
            Dmin(i,j), ...
            status ...
        );

    end

end


%% ============================================================
% 16. COLORS
% ============================================================

colors = lines(nUAV);


%% ============================================================
% 17. PLOT TRAJECTORIES
% ============================================================

figure;

hold on;
grid on;
axis equal;


%% Square

plot( ...
    [0 L L 0 0], ...
    [0 0 L L 0], ...
    'k--', ...
    'LineWidth',1.5, ...
    'DisplayName','Square' ...
);


%% Waypoint circle

theta_circle = linspace(0,2*pi,300);


plot( ...
    C(1) + rho*cos(theta_circle), ...
    C(2) + rho*sin(theta_circle), ...
    'k:', ...
    'LineWidth',1, ...
    'DisplayName','Waypoint region' ...
);


%% UAV trajectories

for i = 1:nUAV


    plot( ...
        Z(1,:,i), ...
        Z(2,:,i), ...
        'LineWidth',2, ...
        'Color',colors(i,:), ...
        'DisplayName',sprintf('UAV %d',i) ...
    );


    %% Initial point

    plot( ...
        S(1,i), ...
        S(2,i), ...
        'o', ...
        'MarkerSize',8, ...
        'MarkerFaceColor',colors(i,:), ...
        'MarkerEdgeColor',colors(i,:), ...
        'HandleVisibility','off' ...
    );


    %% Waypoint

    plot( ...
        W(1,i), ...
        W(2,i), ...
        's', ...
        'MarkerSize',8, ...
        'MarkerFaceColor',colors(i,:), ...
        'MarkerEdgeColor',colors(i,:), ...
        'HandleVisibility','off' ...
    );

end


plot( ...
    C(1), ...
    C(2), ...
    'kx', ...
    'MarkerSize',10, ...
    'LineWidth',2, ...
    'DisplayName','Center' ...
);


xlabel('x [m]');
ylabel('y [m]');

title('Multi-UAV spatial separation trajectories');

legend('Location','bestoutside');


%% ============================================================
% 18. CONTROL POLYGONS
% ============================================================

figure;

hold on;
grid on;
axis equal;


plot( ...
    [0 L L 0 0], ...
    [0 0 L L 0], ...
    'k--', ...
    'LineWidth',1.5 ...
);


for i = 1:nUAV

    P = P_all(:,:,i);


    %% Trajectory

    plot( ...
        Z(1,:,i), ...
        Z(2,:,i), ...
        'LineWidth',2, ...
        'Color',colors(i,:) ...
    );


    %% Control polygon

    plot( ...
        P(1,:), ...
        P(2,:), ...
        '--o', ...
        'LineWidth',1, ...
        'Color',colors(i,:), ...
        'HandleVisibility','off' ...
    );


    %% Waypoint

    plot( ...
        W(1,i), ...
        W(2,i), ...
        's', ...
        'MarkerFaceColor',colors(i,:), ...
        'MarkerEdgeColor',colors(i,:), ...
        'HandleVisibility','off' ...
    );

end


xlabel('x [m]');
ylabel('y [m]');

title('Bezier trajectories and control polygons');


%% ============================================================
% 19. UAV CONFIGURATION AT t = T/2
% ============================================================

figure;

hold on;
grid on;
axis equal;


plot( ...
    [0 L L 0 0], ...
    [0 0 L L 0], ...
    'k--', ...
    'LineWidth',1.5 ...
);


plot( ...
    C(1) + rho*cos(theta_circle), ...
    C(2) + rho*sin(theta_circle), ...
    'k:', ...
    'LineWidth',1 ...
);


for i = 1:nUAV


    plot( ...
        W(1,i), ...
        W(2,i), ...
        'o', ...
        'MarkerSize',12, ...
        'MarkerFaceColor',colors(i,:), ...
        'MarkerEdgeColor',colors(i,:) ...
    );


    text( ...
        W(1,i)+4, ...
        W(2,i)+4, ...
        sprintf('UAV %d',i) ...
    );

end


xlabel('x [m]');
ylabel('y [m]');

title('UAV configuration at t = T/2');


%% ============================================================
% 20. CRITICAL PAIR
% ============================================================

figure;

hold on;
grid on;
axis equal;


for i = 1:nUAV

    plot( ...
        Z(1,:,i), ...
        Z(2,:,i), ...
        'Color',colors(i,:), ...
        'LineWidth',1 ...
    );

end


iCrit = critical_pair(1);

jCrit = critical_pair(2);

kCrit = critical_index;


plot( ...
    Z(1,kCrit,iCrit), ...
    Z(2,kCrit,iCrit), ...
    'o', ...
    'MarkerSize',12, ...
    'MarkerFaceColor',colors(iCrit,:), ...
    'MarkerEdgeColor',colors(iCrit,:) ...
);


plot( ...
    Z(1,kCrit,jCrit), ...
    Z(2,kCrit,jCrit), ...
    'o', ...
    'MarkerSize',12, ...
    'MarkerFaceColor',colors(jCrit,:), ...
    'MarkerEdgeColor',colors(jCrit,:) ...
);


plot( ...
    [ ...
        Z(1,kCrit,iCrit) ...
        Z(1,kCrit,jCrit) ...
    ], ...
    [ ...
        Z(2,kCrit,iCrit) ...
        Z(2,kCrit,jCrit) ...
    ], ...
    'k-', ...
    'LineWidth',2 ...
);


xlabel('x [m]');
ylabel('y [m]');


title( ...
    sprintf( ...
        'Critical pair UAV %d-UAV %d | d = %.2f m | t = %.2f s', ...
        iCrit, ...
        jCrit, ...
        global_min_distance, ...
        critical_time ...
    ) ...
);


%% ============================================================
% 21. AIRSPEED
% ============================================================

figure;

hold on;
grid on;


for i = 1:nUAV

    plot( ...
        t, ...
        Va(i,:), ...
        'LineWidth',1.5, ...
        'Color',colors(i,:), ...
        'DisplayName',sprintf('UAV %d',i) ...
    );

end


yline(18,'k--','V_a min');

yline(25,'k--','V_a max');


xlabel('Time [s]');
ylabel('V_a [m/s]');

title('Airspeed');

legend('Location','bestoutside');


%% ============================================================
% 22. BANK ANGLE
% ============================================================

figure;

hold on;
grid on;


for i = 1:nUAV

    plot( ...
        t, ...
        Phi(i,:), ...
        'LineWidth',1.5, ...
        'Color',colors(i,:), ...
        'DisplayName',sprintf('UAV %d',i) ...
    );

end


yline(0.43,'k--','\Phi max');

yline(-0.43,'k--','\Phi min');


xlabel('Time [s]');
ylabel('\Phi [rad]');

title('Bank angle');

legend('Location','bestoutside');


%% ============================================================
% 23. AIRSPEED ACCELERATION
% ============================================================

figure;

hold on;
grid on;


for i = 1:nUAV

    plot( ...
        t, ...
        Va_dot(i,:), ...
        'LineWidth',1.5, ...
        'Color',colors(i,:), ...
        'DisplayName',sprintf('UAV %d',i) ...
    );

end


xlabel('Time [s]');
ylabel('dV_a / dt [m/s^2]');

title('Airspeed acceleration');

legend('Location','bestoutside');


%% ============================================================
% 24. MINIMUM DISTANCE MATRIX
% ============================================================

figure;


Dplot = Dmin;

Dplot(1:nUAV+1:end) = NaN;


imagesc(Dplot);

axis equal tight;

colorbar;


xlabel('UAV');
ylabel('UAV');

title('Minimum pairwise distance [m]');


xticks(1:nUAV);

yticks(1:nUAV);


%% ============================================================
% 25. MULTI-UAV ANIMATION
% ============================================================

figure;

hold on;
grid on;
axis equal;


margin = 80;


xmin = min(Z(1,:),[],'all');

xmax = max(Z(1,:),[],'all');

ymin = min(Z(2,:),[],'all');

ymax = max(Z(2,:),[],'all');


xlim([ ...
    min(0,xmin)-margin, ...
    max(L,xmax)+margin ...
]);


ylim([ ...
    min(0,ymin)-margin, ...
    max(L,ymax)+margin ...
]);


xlabel('x [m]');
ylabel('y [m]');


%% Square

plot( ...
    [0 L L 0 0], ...
    [0 0 L L 0], ...
    'k--', ...
    'LineWidth',1.5 ...
);


%% Waypoint circle

plot( ...
    C(1) + rho*cos(theta_circle), ...
    C(2) + rho*sin(theta_circle), ...
    'k:', ...
    'LineWidth',1 ...
);


%% Waypoints

for i = 1:nUAV

    plot( ...
        W(1,i), ...
        W(2,i), ...
        's', ...
        'MarkerSize',6, ...
        'MarkerFaceColor',colors(i,:), ...
        'MarkerEdgeColor',colors(i,:) ...
    );

end


%% UAV objects

hUAV = gobjects(nUAV,1);

hTrail = gobjects(nUAV,1);


for i = 1:nUAV


    hTrail(i) = animatedline( ...
        'Color',colors(i,:), ...
        'LineWidth',1.5 ...
    );


    hUAV(i) = plot( ...
        Z(1,1,i), ...
        Z(2,1,i), ...
        'o', ...
        'MarkerSize',10, ...
        'MarkerFaceColor',colors(i,:), ...
        'MarkerEdgeColor',colors(i,:) ...
    );

end


%% ============================================================
% ANIMATION LOOP
% ============================================================

animationStep = 3;


for k = 1:animationStep:N


    for i = 1:nUAV


        %% Trail

        addpoints( ...
            hTrail(i), ...
            Z(1,k,i), ...
            Z(2,k,i) ...
        );


        %% UAV position

        set( ...
            hUAV(i), ...
            'XData',Z(1,k,i), ...
            'YData',Z(2,k,i) ...
        );

    end


    title( ...
        sprintf( ...
            'Multi-UAV spatial separation | t = %.2f s', ...
            t(k) ...
        ) ...
    );


    drawnow;

end