%% ============================================================
% DUBINS CAR - EXERCISE 2
%
% Relax the waypoint constraints:
%
%       ||z(t_j) - w_j|| <= delta
%
% Find delta_bar such that:
%
%       L(delta = 0) <= 1.10 * L(delta_bar)
%
% We choose the limiting value:
%
%       L(delta_bar) ~= L(0)/1.10
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


% Normalized passing times

tw = [ ...
    0;
    0.4;
    0.6;
    0.8;
    1 ...
];


m = size(W,1);


%% ============================================================
% 2. BEZIER PARAMETERS
%
% Same choice as the TP:
%
% n = 5  ->  6 control points
% ============================================================

n = 5;

Ncp = n + 1;


%% ============================================================
% 3. WAYPOINT MATRIX
%
% z(t_j) = Bwp * P
% ============================================================

Bwp = zeros(m,Ncp);


for j = 1:m

    for i = 0:n

        Bwp(j,i+1) = ...
            bernstein(i,n,tw(j));

    end

end


%% ============================================================
% 4. ENERGY MATRIX
%
% J = integral ||z_dot(t)||^2 dt
% ============================================================

E = zeros(n,Ncp);


for i = 1:n

    E(i,i) = -1;

    E(i,i+1) = 1;

end


%% Gram matrix

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


%% Energy matrix for one coordinate

Q = ...
    n^2 ...
    * E' ...
    * G ...
    * E;


%% x and y

H = blkdiag(Q,Q);


%% Small numerical regularization

H = H + ...
    1e-10*eye(size(H));


%% ============================================================
% 5. EXACT PROBLEM: delta = 0
%
% First calculate the reference trajectory.
% ============================================================

Aeq = [ ...
    Bwp,             zeros(m,Ncp);
    zeros(m,Ncp),    Bwp ...
];


beq = [ ...
    W(:,1);
    W(:,2) ...
];


%% quadprog objective:
%
% min 1/2 q'Hq
%
% q = [Px ; Py]

Hqp = 2*H;

f = zeros(2*Ncp,1);


optionsQP = optimoptions( ...
    'quadprog', ...
    'Display','off' ...
);


[q_exact,~,exitflag] = quadprog( ...
    Hqp, ...
    f, ...
    [],[], ...
    Aeq, ...
    beq, ...
    [],[], ...
    [], ...
    optionsQP ...
);


if exitflag <= 0

    error('Exact optimization problem failed.');

end


%% ============================================================
% 6. RECONSTRUCT EXACT CONTROL POINTS
% ============================================================

P_exact = [ ...
    q_exact(1:Ncp)';
    q_exact(Ncp+1:end)' ...
];


%% ============================================================
% 7. EXACT PATH LENGTH
% ============================================================

L0 = pathLength( ...
    P_exact, ...
    n ...
);


fprintf('\n');
fprintf('============================================\n');
fprintf('EXACT PROBLEM - delta = 0\n');
fprintf('============================================\n');

fprintf( ...
    'Path length L0 = %.8f\n', ...
    L0 ...
);


%% ============================================================
% 8. TARGET LENGTH
%
% L0 = 1.10 * L(delta_bar)
%
% Therefore:
%
% L(delta_bar) = L0 / 1.10
% ============================================================

L_target = L0 / 1.10;


fprintf( ...
    'Target relaxed path length = %.8f\n', ...
    L_target ...
);


%% ============================================================
% 9. FIND AN UPPER BOUND FOR delta
%
% Increase delta until the trajectory becomes
% more than 10% shorter.
% ============================================================

delta_low = 0;

delta_high = 0.05;


optionsFMINCON = optimoptions( ...
    'fmincon', ...
    'Display','off', ...
    'Algorithm','interior-point', ...
    'MaxIterations',2000, ...
    'OptimalityTolerance',1e-10, ...
    'ConstraintTolerance',1e-9 ...
);


while true


    [q_test,success] = solveRelaxedProblem( ...
        delta_high, ...
        q_exact, ...
        H, ...
        Bwp, ...
        W, ...
        optionsFMINCON ...
    );


    if ~success

        error('Relaxed optimization failed.');

    end


    P_test = [ ...
        q_test(1:Ncp)';
        q_test(Ncp+1:end)' ...
    ];


    L_test = pathLength( ...
        P_test, ...
        n ...
    );


    fprintf( ...
        'delta = %.6f  ->  length = %.8f\n', ...
        delta_high, ...
        L_test ...
    );


    %% Stop when we cross the 10% boundary

    if L_test < L_target

        break;

    end


    delta_high = ...
        2*delta_high;


end


%% ============================================================
% 10. BINARY SEARCH FOR delta_bar
% ============================================================

Niterations = 35;


for iter = 1:Niterations


    delta_mid = ...
        (delta_low + delta_high)/2;


    [q_mid,success] = solveRelaxedProblem( ...
        delta_mid, ...
        q_exact, ...
        H, ...
        Bwp, ...
        W, ...
        optionsFMINCON ...
    );


    if ~success

        error('Optimization failed during binary search.');

    end


    P_mid = [ ...
        q_mid(1:Ncp)';
        q_mid(Ncp+1:end)' ...
    ];


    L_mid = pathLength( ...
        P_mid, ...
        n ...
    );


    %% --------------------------------------------------------
    % If trajectory is still within the 10% requirement,
    % we can increase delta.
    % ---------------------------------------------------------

    if L_mid >= L_target

        delta_low = delta_mid;

    else

        delta_high = delta_mid;

    end

end


%% ============================================================
% 11. FINAL delta_bar
% ============================================================

delta_bar = delta_low;


[q_bar,success] = solveRelaxedProblem( ...
    delta_bar, ...
    q_exact, ...
    H, ...
    Bwp, ...
    W, ...
    optionsFMINCON ...
);


if ~success

    error('Final optimization failed.');

end


P_bar = [ ...
    q_bar(1:Ncp)';
    q_bar(Ncp+1:end)' ...
];


L_bar = pathLength( ...
    P_bar, ...
    n ...
);


%% ============================================================
% 12. RESULTS
% ============================================================

ratio = L0/L_bar;

percentage = ...
    (ratio - 1)*100;


fprintf('\n');
fprintf('============================================\n');
fprintf('FINAL RESULTS\n');
fprintf('============================================\n');


fprintf( ...
    'L(delta = 0)       = %.8f\n', ...
    L0 ...
);


fprintf( ...
    'delta_bar          = %.8f\n', ...
    delta_bar ...
);


fprintf( ...
    'L(delta_bar)       = %.8f\n', ...
    L_bar ...
);


fprintf( ...
    'L0 / L(delta_bar)  = %.8f\n', ...
    ratio ...
);


fprintf( ...
    'Difference         = %.4f %%\n', ...
    percentage ...
);


%% ============================================================
% 13. WAYPOINT ERRORS
% ============================================================

fprintf('\n');
fprintf('============================================\n');
fprintf('WAYPOINT DISTANCES FOR delta_bar\n');
fprintf('============================================\n');


for j = 1:m


    Bs = zeros(Ncp,1);


    for i = 0:n

        Bs(i+1) = ...
            bernstein(i,n,tw(j));

    end


    z_j = ...
        P_bar * Bs;


    error_j = norm( ...
        z_j ...
        - W(j,:)' ...
    );


    fprintf( ...
        'Waypoint %d : distance = %.8f   delta = %.8f\n', ...
        j, ...
        error_j, ...
        delta_bar ...
    );

end


%% ============================================================
% 14. EVALUATE TRAJECTORIES
% ============================================================

tplot = linspace( ...
    0, ...
    1, ...
    2000 ...
);


Z_exact = evaluateBezier( ...
    P_exact, ...
    n, ...
    tplot ...
);


Z_bar = evaluateBezier( ...
    P_bar, ...
    n, ...
    tplot ...
);


%% ============================================================
% 15. PLOT EXACT VS RELAXED TRAJECTORY
% ============================================================

figure;

hold on;
grid on;
axis equal;


%% Exact trajectory

plot( ...
    Z_exact(1,:), ...
    Z_exact(2,:), ...
    'LineWidth',2, ...
    'DisplayName','\delta = 0' ...
);


%% Relaxed trajectory

plot( ...
    Z_bar(1,:), ...
    Z_bar(2,:), ...
    'LineWidth',2, ...
    'DisplayName', ...
    sprintf('\\delta = %.4f',delta_bar) ...
);


%% Waypoints

plot( ...
    W(:,1), ...
    W(:,2), ...
    'ks', ...
    'MarkerSize',8, ...
    'MarkerFaceColor','k', ...
    'DisplayName','Waypoints' ...
);


%% ============================================================
% NEIGHBORHOOD CIRCLES
% ============================================================

theta = linspace( ...
    0, ...
    2*pi, ...
    300 ...
);


for j = 1:m


    x_circle = ...
        W(j,1) ...
        + delta_bar*cos(theta);


    y_circle = ...
        W(j,2) ...
        + delta_bar*sin(theta);


    plot( ...
        x_circle, ...
        y_circle, ...
        'k:', ...
        'HandleVisibility','off' ...
    );

end


xlabel('x');

ylabel('y');


title( ...
    'Exact and relaxed waypoint trajectories' ...
);


legend( ...
    'Location','best' ...
);


%% ============================================================
% 16. PATH LENGTH AS FUNCTION OF delta
% ============================================================

delta_values = linspace( ...
    0, ...
    1.3*delta_bar, ...
    40 ...
);


length_values = zeros( ...
    size(delta_values) ...
);


length_values(1) = L0;


for k = 2:length(delta_values)


    delta = delta_values(k);


    [q_temp,success] = solveRelaxedProblem( ...
        delta, ...
        q_exact, ...
        H, ...
        Bwp, ...
        W, ...
        optionsFMINCON ...
    );


    if success


        P_temp = [ ...
            q_temp(1:Ncp)';
            q_temp(Ncp+1:end)' ...
        ];


        length_values(k) = ...
            pathLength( ...
                P_temp, ...
                n ...
            );


    else

        length_values(k) = NaN;

    end

end


%% ============================================================
% 17. PLOT LENGTH VS delta
% ============================================================

figure;

hold on;
grid on;


plot( ...
    delta_values, ...
    length_values, ...
    '-o', ...
    'LineWidth',1.5 ...
);


%% Target line

yline( ...
    L_target, ...
    'k--', ...
    '10% threshold' ...
);


%% delta_bar

xline( ...
    delta_bar, ...
    'r--', ...
    '\delta bar' ...
);


xlabel('\delta');

ylabel('Total path length');


title( ...
    'Path length versus waypoint relaxation' ...
);


%% ============================================================
% LOCAL FUNCTIONS
% ============================================================


%% ============================================================
% Bernstein polynomial
% ============================================================

function value = bernstein(i,n,t)

    value = ...
        nchoosek(n,i) ...
        .* (1-t).^(n-i) ...
        .* t.^i;

end


%% ============================================================
% Solve relaxed optimization problem
% ============================================================

function [q_opt,success] = solveRelaxedProblem( ...
    delta, ...
    q0, ...
    H, ...
    Bwp, ...
    W, ...
    options ...
)


    objective = @(q) ...
        q' * H * q;


    nonlinearConstraints = @(q) ...
        waypointConstraints( ...
            q, ...
            delta, ...
            Bwp, ...
            W ...
        );


    [q_opt,~,exitflag] = fmincon( ...
        objective, ...
        q0, ...
        [],[], ...
        [],[], ...
        [],[], ...
        nonlinearConstraints, ...
        options ...
    );


    success = ...
        exitflag > 0;

end


%% ============================================================
% Neighborhood constraints
%
% ||z(t_j) - w_j|| <= delta
%
% fmincon expects:
%
% c(q) <= 0
%
% so:
%
% ||z-w||^2 - delta^2 <= 0
% ============================================================

function [c,ceq] = waypointConstraints( ...
    q, ...
    delta, ...
    Bwp, ...
    W ...
)


    m = size(W,1);

    Ncp = size(Bwp,2);


    Px = q(1:Ncp);

    Py = q(Ncp+1:end);


    c = zeros(m,1);


    for j = 1:m


        xj = ...
            Bwp(j,:) * Px;


        yj = ...
            Bwp(j,:) * Py;


        dx = ...
            xj ...
            - W(j,1);


        dy = ...
            yj ...
            - W(j,2);


        c(j) = ...
            dx^2 ...
            + dy^2 ...
            - delta^2;

    end


    ceq = [];

end


%% ============================================================
% Evaluate Bezier trajectory
% ============================================================

function Z = evaluateBezier( ...
    P, ...
    n, ...
    t ...
)


    Z = zeros( ...
        2, ...
        length(t) ...
    );


    for k = 1:length(t)

        for i = 0:n

            Z(:,k) = ...
                Z(:,k) ...
                + P(:,i+1) ...
                * bernstein( ...
                    i, ...
                    n, ...
                    t(k) ...
                  );

        end

    end

end


%% ============================================================
% Calculate path length
%
% L = integral ||z_dot(t)|| dt
% ============================================================

function L = pathLength(P,n)


    Nt = 5000;


    t = linspace( ...
        0, ...
        1, ...
        Nt ...
    );


    ZD = zeros( ...
        2, ...
        Nt ...
    );


    for k = 1:Nt

        for i = 0:n-1

            ZD(:,k) = ...
                ZD(:,k) ...
                + n ...
                * ( ...
                    P(:,i+2) ...
                    - P(:,i+1) ...
                  ) ...
                * bernstein( ...
                    i, ...
                    n-1, ...
                    t(k) ...
                  );

        end

    end


    speed = sqrt( ...
        ZD(1,:).^2 ...
        + ZD(2,:).^2 ...
    );


    L = trapz( ...
        t, ...
        speed ...
    );

end