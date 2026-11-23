%% ============================================================
% MULTI-UAV TRAJECTORY PLANNING
%
% 8 UAVs - 2D Swapping Square
%
% Bezier parametrization - degree 6
%
% Approach 1:
%   - Same waypoint for all UAVs
%   - Different waypoint crossing times
%   - Different initial/final headings to avoid collinear paths
%
% Flat output:
%
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

d_safe = 10;                % Required safety distance [m]


%% ============================================================
% HEADING DEVIATION
%
% Increase this value to obtain more curved trajectories.
%
% ============================================================

deltaPsi = deg2rad(5);


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
% 4. COMMON WAYPOINT
% ============================================================

w = [ ...
    L/2;
    L/2 ...
];


%% ============================================================
% 5. WAYPOINT CROSSING TIMES
%
% All UAVs pass through the same waypoint,
% but at different times.
%
% ============================================================

tw = linspace( ...
    0.25*T, ...
    0.75*T, ...
    nUAV ...
);

tau_w_all = tw/T;


%% ============================================================
% 6. INITIAL / FINAL CONDITIONS
% ============================================================

Va_i = V0*ones(1,nUAV);

Va_f = V0*ones(1,nUAV);


% Initial / final bank angles
Phi_i = zeros(1,nUAV);

Phi_f = zeros(1,nUAV);


% Initial / final heading
Psi_i = zeros(1,nUAV);

Psi_f = zeros(1,nUAV);


%% ============================================================
% CURVATURE SIDE
%
% Opposite UAVs receive opposite deviations.
%
% UAV 1 / UAV 5
% UAV 2 / UAV 6
% UAV 3 / UAV 7
% UAV 4 / UAV 8
%
% ============================================================

side = [ ...
     1  1  1  1 ...
    -1 -1 -1 -1 ...
];


%% ============================================================
% CALCULATE INITIAL / FINAL HEADINGS
% ============================================================

for i = 1:nUAV

    zi = S(:,i);

    zf = S(:,destination(i));


    % Direct orientation toward destination
    direction = zf - zi;

    psi_base = atan2( ...
        direction(2), ...
        direction(1) ...
    );


    % Initial heading
    Psi_i(i) = ...
        psi_base ...
        + side(i)*deltaPsi;


    % Final heading
    Psi_f(i) = ...
        psi_base ...
        - side(i)*deltaPsi;

end


%% ============================================================
% 7. BERNSTEIN BASIS
% ============================================================

B = @(i,n,s) ...
    nchoosek(n,i) .* ...
    (1-s).^(n-i) .* ...
    s.^i;


%% ============================================================
% 8. MEMORY ALLOCATION
% ============================================================

% Bezier control points
%
% P_all(:,:,uav)
%
% 2 coordinates
% 7 control points
% 8 UAVs

P_all = zeros(2,7,nUAV);


% Flat output and derivatives

Z   = zeros(2,N,nUAV);

ZD  = zeros(2,N,nUAV);

ZDD = zeros(2,N,nUAV);


% Reconstructed variables

Psi = zeros(nUAV,N);

Va = zeros(nUAV,N);

Phi = zeros(nUAV,N);

Va_dot = zeros(nUAV,N);


%% ============================================================
% 9. GENERATE BEZIER TRAJECTORY FOR EACH UAV
% ============================================================

for uav = 1:nUAV


    %% --------------------------------------------------------
    % Initial / final positions
    % ---------------------------------------------------------

    zi = S(:,uav);

    zf = S(:,destination(uav));


    %% --------------------------------------------------------
    % Initial velocity
    % ---------------------------------------------------------

    vi = Va_i(uav) * [ ...
        cos(Psi_i(uav));
        sin(Psi_i(uav)) ...
    ];


    %% --------------------------------------------------------
    % Final velocity
    % ---------------------------------------------------------

    vf = Va_f(uav) * [ ...
        cos(Psi_f(uav));
        sin(Psi_f(uav)) ...
    ];


    %% --------------------------------------------------------
    % Initial acceleration
    %
    % Va_dot = 0 at the boundary
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
    % BEZIER CONTROL POINTS
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
    % EXACT WAYPOINT CONDITION
    %
    % z(tw) = w
    %
    % Determines P3
    % ========================================================

    tau_w = tau_w_all(uav);


    B0 = B(0,6,tau_w);
    B1 = B(1,6,tau_w);
    B2 = B(2,6,tau_w);
    B3 = B(3,6,tau_w);
    B4 = B(4,6,tau_w);
    B5 = B(5,6,tau_w);
    B6 = B(6,6,tau_w);


    P3 = ( ...
          w ...
        - P0*B0 ...
        - P1*B1 ...
        - P2*B2 ...
        - P4*B4 ...
        - P5*B5 ...
        - P6*B6 ...
        ) / B3;


    %% --------------------------------------------------------
    % Complete control-point matrix
    % ---------------------------------------------------------

    P = [ ...
        P0 P1 P2 P3 P4 P5 P6 ...
    ];


    P_all(:,:,uav) = P;


    %% ========================================================
    % EVALUATE TRAJECTORY
    % ========================================================

    for k = 1:N

        s = tau(k);


        %% ----------------------------------------------------
        % Position
        % -----------------------------------------------------

        z = zeros(2,1);


        for j = 0:6

            z = z ...
                + P(:,j+1)*B(j,6,s);

        end


        Z(:,k,uav) = z;


        %% ----------------------------------------------------
        % First derivative
        % -----------------------------------------------------

        zd = zeros(2,1);


        for j = 0:5

            zd = zd ...
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

            zdd = zdd ...
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
        + ZD(2,:,uav).^2 ...
    );


    %% Bank angle

    Phi(uav,:) = atan2( ...
        ZDD(2,:,uav).*ZD(1,:,uav) ...
        - ZD(2,:,uav).*ZDD(1,:,uav), ...
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
% 10. EXACT WAYPOINT VERIFICATION
% ============================================================

fprintf('\n');
fprintf('============================================\n');
fprintf('EXACT WAYPOINT VERIFICATION\n');
fprintf('============================================\n');


for uav = 1:nUAV

    P = P_all(:,:,uav);

    tau_w = tw(uav)/T;


    z_wp = zeros(2,1);


    for j = 0:6

        z_wp = z_wp ...
            + P(:,j+1)*B(j,6,tau_w);

    end


    error_wp = norm( ...
        z_wp ...
        - w ...
    );


    fprintf( ...
        'UAV %d | tw = %.3f s | error = %.12e m\n', ...
        uav, ...
        tw(uav), ...
        error_wp ...
    );

end


%% ============================================================
% 11. COLLISION CHECK
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
% 12. PRINT COLLISION RESULTS
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
% 13. PRINT ALL PAIRWISE MINIMUM DISTANCES
% ============================================================

fprintf('\n');
fprintf('============================================\n');
fprintf('PAIRWISE MINIMUM DISTANCES\n');
fprintf('============================================\n');


for i = 1:nUAV-1

    for j = i+1:nUAV

        if Dmin(i,j) < d_safe

            status = 'WARNING';

        else

            status = 'SAFE';

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
% 14. PLOT TRAJECTORIES
% ============================================================

colors = lines(nUAV);


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


%% Trajectories

for i = 1:nUAV

    plot( ...
        Z(1,:,i), ...
        Z(2,:,i), ...
        'LineWidth',2, ...
        'Color',colors(i,:), ...
        'DisplayName',sprintf('UAV %d',i) ...
    );


    plot( ...
        S(1,i), ...
        S(2,i), ...
        'o', ...
        'MarkerSize',8, ...
        'MarkerFaceColor',colors(i,:), ...
        'MarkerEdgeColor',colors(i,:), ...
        'HandleVisibility','off' ...
    );

end


%% Common waypoint

plot( ...
    w(1), ...
    w(2), ...
    'ks', ...
    'MarkerSize',11, ...
    'LineWidth',2, ...
    'DisplayName','Common waypoint' ...
);


xlabel('x [m]');
ylabel('y [m]');

title( ...
    sprintf( ...
        'Multi-UAV trajectories - \\Delta\\Psi = %.1f deg', ...
        rad2deg(deltaPsi) ...
    ) ...
);

legend('Location','bestoutside');


%% ============================================================
% 15. CONTROL POLYGONS + TRAJECTORIES
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

end


plot( ...
    w(1), ...
    w(2), ...
    'ks', ...
    'MarkerSize',10, ...
    'LineWidth',2 ...
);


xlabel('x [m]');
ylabel('y [m]');

title('Bezier trajectories and control polygons');


%% ============================================================
% 16. PLOT CRITICAL COLLISION CONFIGURATION
% ============================================================

figure;

hold on;
grid on;
axis equal;


%% Plot trajectories

for i = 1:nUAV

    plot( ...
        Z(1,:,i), ...
        Z(2,:,i), ...
        'Color',colors(i,:), ...
        'LineWidth',1 ...
    );

end


i = critical_pair(1);

j = critical_pair(2);

k = critical_index;


%% Critical UAV positions

plot( ...
    Z(1,k,i), ...
    Z(2,k,i), ...
    'o', ...
    'MarkerSize',12, ...
    'MarkerFaceColor',colors(i,:), ...
    'LineWidth',2 ...
);


plot( ...
    Z(1,k,j), ...
    Z(2,k,j), ...
    'o', ...
    'MarkerSize',12, ...
    'MarkerFaceColor',colors(j,:), ...
    'LineWidth',2 ...
);


%% Line between critical UAVs

plot( ...
    [Z(1,k,i) Z(1,k,j)], ...
    [Z(2,k,i) Z(2,k,j)], ...
    'k-', ...
    'LineWidth',2 ...
);


xlabel('x [m]');
ylabel('y [m]');


title( ...
    sprintf( ...
        'Critical pair UAV %d-UAV %d | d = %.2f m | t = %.2f s', ...
        i, ...
        j, ...
        global_min_distance, ...
        critical_time ...
    ) ...
);


%% ============================================================
% 17. AIRSPEED
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


yline( ...
    18, ...
    'k--', ...
    'V_a min' ...
);


yline( ...
    25, ...
    'k--', ...
    'V_a max' ...
);


xlabel('Time [s]');
ylabel('V_a [m/s]');

title('Airspeed');

legend('Location','bestoutside');


%% ============================================================
% 18. BANK ANGLE
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


yline( ...
    0.43, ...
    'k--', ...
    '\Phi max' ...
);


yline( ...
    -0.43, ...
    'k--', ...
    '\Phi min' ...
);


xlabel('Time [s]');
ylabel('\Phi [rad]');

title('Bank angle');

legend('Location','bestoutside');


%% ============================================================
% 19. AIRSPEED ACCELERATION
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
% 20. MINIMUM DISTANCE MATRIX
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
% 21. MULTI-UAV ANIMATION
% ============================================================

figure;

hold on;
grid on;
axis equal;


margin = 100;


% Include possible trajectory excursion outside square

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


%% Waypoint

plot( ...
    w(1), ...
    w(2), ...
    'ks', ...
    'MarkerSize',10, ...
    'LineWidth',2 ...
);


%% UAV graphics

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


        %% Add trajectory point

        addpoints( ...
            hTrail(i), ...
            Z(1,k,i), ...
            Z(2,k,i) ...
        );


        %% Move UAV

        set( ...
            hUAV(i), ...
            'XData',Z(1,k,i), ...
            'YData',Z(2,k,i) ...
        );

    end


    title( ...
        sprintf( ...
            'Multi-UAV swapping | t = %.2f s', ...
            t(k) ...
        ) ...
    );


    drawnow;

end