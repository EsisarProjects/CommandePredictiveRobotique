%% ============================================================
% DUBINS CAR - EXERCISE 1
%
% For a given collection of m waypoints:
%
% Plot the total path length as a function of
% n + 1 >= m, the number of control points.
%
% Cost:
%
%       J = integral ||z_dot(t)||^2 dt
%
% Constraints:
%
%       z(t_j) = w_j
%
% ============================================================

clear;
clc;
close all;


%% ============================================================
% 1. WAYPOINTS
% ============================================================

W = [ ...
    0 0;
    1 2;
    3 1;
    4 2;
    5 1 ...
];


% Normalized waypoint times

tw = [ ...
    0;
    0.4;
    0.6;
    0.8;
    1 ...
];


% Number of waypoints

m = size(W,1);


%% ============================================================
% 2. NUMBER OF CONTROL POINTS TO TEST
%
% Requirement:
%
%       n + 1 >= m
%
% ============================================================

Ncp_min = m;

Ncp_max = 12;

controlPointValues = Ncp_min:Ncp_max;


%% ============================================================
% 3. MEMORY
% ============================================================

pathLengths = zeros( ...
    size(controlPointValues) ...
);


% Store solutions for later plotting

P_all = cell( ...
    size(controlPointValues) ...
);


%% ============================================================
% 4. OPTIMIZATION LOOP
% ============================================================

for test = 1:length(controlPointValues)


    %% --------------------------------------------------------
    % Number of control points
    % ---------------------------------------------------------

    Ncp = controlPointValues(test);


    % Bezier degree

    n = Ncp - 1;


    fprintf('\n');
    fprintf('====================================\n');
    fprintf('Control points : %d\n',Ncp);
    fprintf('Bezier degree  : %d\n',n);
    fprintf('====================================\n');


    %% ========================================================
    % 5. WAYPOINT CONSTRAINT MATRIX
    %
    % z(t_j) = sum Pi B_i,n(t_j)
    %
    % ========================================================

    Bwp = zeros(m,Ncp);


    for j = 1:m

        for i = 0:n

            Bwp(j,i+1) = ...
                bernstein(i,n,tw(j));

        end

    end


    %% ========================================================
    % VARIABLES
    %
    % q =
    %
    % [Px0 ... Pxn Py0 ... Pyn]'
    %
    % ========================================================


    %% ========================================================
    % EQUALITY CONSTRAINTS
    %
    % Bwp * Px = Wx
    % Bwp * Py = Wy
    %
    % ========================================================

    Aeq = [ ...
        Bwp,             zeros(m,Ncp);
        zeros(m,Ncp),    Bwp ...
    ];


    beq = [ ...
        W(:,1);
        W(:,2) ...
    ];


    %% ========================================================
    % 6. DERIVATIVE DIFFERENCE MATRIX
    %
    % D_i = P_(i+1) - P_i
    %
    % ========================================================

    E = zeros(n,Ncp);


    for i = 1:n

        E(i,i)   = -1;

        E(i,i+1) = 1;

    end


    %% ========================================================
    % 7. GRAM MATRIX OF BERNSTEIN POLYNOMIALS
    %
    % G(i,k) =
    %
    % integral_0^1 B_i,n-1(t) B_k,n-1(t) dt
    %
    % ========================================================

    G = zeros(n,n);


    p = n - 1;


    for i = 0:p

        for k = 0:p

            G(i+1,k+1) = ...
                nchoosek(p,i) ...
                * nchoosek(p,k) ...
                * factorial(i+k) ...
                * factorial(2*p-i-k) ...
                / factorial(2*p+1);

        end

    end


    %% ========================================================
    % 8. ENERGY MATRIX
    %
    % J =
    %
    % integral ||z_dot||^2 dt
    %
    % z_dot =
    %
    % n sum (P_i+1 - P_i) B_i,n-1
    %
    % ========================================================

    Q = ...
        n^2 ...
        * E' ...
        * G ...
        * E;


    %% --------------------------------------------------------
    % Both x and y coordinates
    % ---------------------------------------------------------

    H = blkdiag( ...
        2*Q, ...
        2*Q ...
    );


    % Small numerical regularization

    H = H + ...
        1e-10*eye(size(H));


    f = zeros( ...
        2*Ncp, ...
        1 ...
    );


    %% ========================================================
    % 9. SOLVE QUADRATIC PROGRAM
    %
    % quadprog solves:
    %
    % min 1/2 q' H q + f' q
    %
    % subject to:
    %
    % Aeq q = beq
    %
    % ========================================================

    options = optimoptions( ...
        'quadprog', ...
        'Display','off' ...
    );


    [q_opt,~,exitflag] = quadprog( ...
        H, ...
        f, ...
        [],[], ...
        Aeq, ...
        beq, ...
        [],[], ...
        [], ...
        options ...
    );


    if exitflag <= 0

        warning( ...
            'Optimization failed for %d control points.', ...
            Ncp ...
        );

        pathLengths(test) = NaN;

        continue;

    end


    %% ========================================================
    % 10. RECONSTRUCT CONTROL POINT MATRIX
    % ========================================================

    Px = q_opt(1:Ncp);

    Py = q_opt(Ncp+1:end);


    P = [ ...
        Px';
        Py' ...
    ];


    P_all{test} = P;


    %% ========================================================
    % 11. EVALUATE OPTIMAL TRAJECTORY
    % ========================================================

    Nt = 5000;


    t = linspace( ...
        0, ...
        1, ...
        Nt ...
    );


    Z  = zeros(2,Nt);

    ZD = zeros(2,Nt);


    for k = 1:Nt

        tk = t(k);


        %% ----------------------------------------------------
        % Position
        % -----------------------------------------------------

        for i = 0:n

            Z(:,k) = ...
                Z(:,k) ...
                + P(:,i+1) ...
                * bernstein(i,n,tk);

        end


        %% ----------------------------------------------------
        % First derivative
        % -----------------------------------------------------

        for i = 0:n-1

            ZD(:,k) = ...
                ZD(:,k) ...
                + n ...
                * ( ...
                    P(:,i+2) ...
                    - P(:,i+1) ...
                  ) ...
                * bernstein(i,n-1,tk);

        end

    end


    %% ========================================================
    % 12. PATH LENGTH
    %
    % l = integral ||z_dot(t)|| dt
    %
    % ========================================================

    speed = sqrt( ...
        ZD(1,:).^2 ...
        + ZD(2,:).^2 ...
    );


    pathLength = trapz( ...
        t, ...
        speed ...
    );


    pathLengths(test) = pathLength;


    fprintf( ...
        'Optimal path length = %.6f\n', ...
        pathLength ...
    );

end


%% ============================================================
% 13. RESULTS
% ============================================================

fprintf('\n\n');

fprintf('============================================\n');

fprintf('FINAL RESULTS\n');

fprintf('============================================\n');


for k = 1:length(controlPointValues)

    fprintf( ...
        '%2d control points | length = %.6f\n', ...
        controlPointValues(k), ...
        pathLengths(k) ...
    );

end


%% ============================================================
% 14. PLOT PATH LENGTH
% ============================================================

figure;

plot( ...
    controlPointValues, ...
    pathLengths, ...
    '-o', ...
    'LineWidth',2, ...
    'MarkerSize',8 ...
);


grid on;


xlabel( ...
    'Number of control points (n+1)' ...
);


ylabel( ...
    'Total path length' ...
);


title( ...
    'Path length vs number of Bezier control points' ...
);


%% ============================================================
% 15. PLOT ALL OPTIMAL TRAJECTORIES
% ============================================================

figure;

hold on;

grid on;

axis equal;


colors = lines( ...
    length(controlPointValues) ...
);


for test = 1:length(controlPointValues)


    P = P_all{test};


    if isempty(P)

        continue;

    end


    Ncp = controlPointValues(test);

    n = Ncp - 1;


    tplot = linspace( ...
        0, ...
        1, ...
        1000 ...
    );


    Zplot = zeros( ...
        2, ...
        length(tplot) ...
    );


    for k = 1:length(tplot)

        for i = 0:n

            Zplot(:,k) = ...
                Zplot(:,k) ...
                + P(:,i+1) ...
                * bernstein( ...
                    i, ...
                    n, ...
                    tplot(k) ...
                  );

        end

    end


    plot( ...
        Zplot(1,:), ...
        Zplot(2,:), ...
        'LineWidth',1.5, ...
        'Color',colors(test,:), ...
        'DisplayName', ...
        sprintf( ...
            '%d control points', ...
            Ncp ...
        ) ...
    );

end


%% Waypoints

plot( ...
    W(:,1), ...
    W(:,2), ...
    'ks', ...
    'MarkerSize',9, ...
    'MarkerFaceColor','k', ...
    'DisplayName','Waypoints' ...
);


xlabel('x');

ylabel('y');


title( ...
    'Optimal trajectories for different numbers of control points' ...
);


legend( ...
    'Location','bestoutside' ...
);


%% ============================================================
% LOCAL FUNCTION
% ============================================================

function value = bernstein(i,n,t)

    value = ...
        nchoosek(n,i) ...
        .* (1-t).^(n-i) ...
        .* t.^i;

end