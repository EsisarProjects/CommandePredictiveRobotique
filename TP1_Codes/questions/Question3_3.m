%% ============================================================
% DUBINS CAR - EXERCISE 3
%
% Add the velocity constraint:
%
%       ||z_dot(t)|| <= rho
%
% Find the minimum rho for which the optimization
% problem remains feasible.
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
% ============================================================

n = 5;

Ncp = n + 1;


%% ============================================================
% 3. WAYPOINT MATRIX
% ============================================================

Bwp = zeros(m,Ncp);


for j = 1:m

    for i = 0:n

        Bwp(j,i+1) = ...
            bernstein(i,n,tw(j));

    end

end


%% ============================================================
% 4. EQUALITY CONSTRAINTS
%
% q = [Px ; Py]
% ============================================================

Aeq = [ ...
    Bwp,             zeros(m,Ncp);
    zeros(m,Ncp),    Bwp ...
];


beq = [ ...
    W(:,1);
    W(:,2) ...
];


%% ============================================================
% 5. DIFFERENCE MATRIX
%
% Pi+1 - Pi
% ============================================================

E = zeros(n,Ncp);


for i = 1:n

    E(i,i) = -1;

    E(i,i+1) = 1;

end


%% ============================================================
% 6. GRAM MATRIX
% ============================================================

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


%% ============================================================
% 7. ENERGY MATRIX
%
% J = integral ||z_dot||^2 dt
% ============================================================

Q = ...
    n^2 ...
    * E' ...
    * G ...
    * E;


H = blkdiag(Q,Q);


H = H + ...
    1e-10*eye(size(H));


%% ============================================================
% 8. SOLVE ORIGINAL UNCONSTRAINED PROBLEM
% ============================================================

Hqp = 2*H;

f = zeros(2*Ncp,1);


optionsQP = optimoptions( ...
    'quadprog', ...
    'Display','off' ...
);


[q_free,~,exitflag] = quadprog( ...
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

    error('Original optimization failed.');

end


%% ============================================================
% 9. RECONSTRUCT CONTROL POINTS
% ============================================================

P_free = [ ...
    q_free(1:Ncp)';
    q_free(Ncp+1:end)' ...
];


%% ============================================================
% 10. MAXIMUM SPEED OF UNCONSTRAINED SOLUTION
% ============================================================

Nverify = 5000;


t_verify = linspace( ...
    0, ...
    1, ...
    Nverify ...
);


speed_free = zeros( ...
    1, ...
    Nverify ...
);


for k = 1:Nverify


    zd = bezierVelocity( ...
        P_free, ...
        n, ...
        t_verify(k) ...
    );


    speed_free(k) = norm(zd);

end


rho_free = max(speed_free);


fprintf('\n');
fprintf('============================================\n');
fprintf('UNCONSTRAINED SOLUTION\n');
fprintf('============================================\n');


fprintf( ...
    'Maximum unconstrained speed = %.8f\n', ...
    rho_free ...
);


%% ============================================================
% 11. THEORETICAL LOWER BOUND
%
% rho >= distance / delta_t
% ============================================================

rho_lower_bound = 0;


for j = 1:m-1


    distance = norm( ...
        W(j+1,:) ...
        - W(j,:) ...
    );


    delta_t = ...
        tw(j+1) ...
        - tw(j);


    rho_segment = ...
        distance ...
        / delta_t;


    fprintf( ...
        'Segment %d -> %d : minimum average speed = %.8f\n', ...
        j, ...
        j+1, ...
        rho_segment ...
    );


    rho_lower_bound = max( ...
        rho_lower_bound, ...
        rho_segment ...
    );

end


fprintf('\n');

fprintf( ...
    'Theoretical lower bound = %.8f\n', ...
    rho_lower_bound ...
);


%% ============================================================
% 12. VELOCITY-CONSTRAINT GRID
%
% The continuous condition is approximated using a
% sufficiently dense grid.
% ============================================================

Nconstraint = 250;


t_constraint = linspace( ...
    0, ...
    1, ...
    Nconstraint ...
);


%% ============================================================
% 13. FMINCON OPTIONS
% ============================================================

optionsFMINCON = optimoptions( ...
    'fmincon', ...
    'Display','off', ...
    'Algorithm','interior-point', ...
    'MaxIterations',3000, ...
    'MaxFunctionEvaluations',100000, ...
    'ConstraintTolerance',1e-8, ...
    'OptimalityTolerance',1e-9, ...
    'StepTolerance',1e-12 ...
);


%% ============================================================
% 14. BINARY SEARCH FOR rho_min
%
% Lower bound:
%
%       rho_lower_bound
%
% Upper bound:
%
%       rho_free
%
% The unconstrained solution is automatically feasible
% for rho = rho_free.
% ============================================================

rho_low = rho_lower_bound;

rho_high = rho_free;


% Current feasible initial solution

q_feasible = q_free;


Niterations = 35;


for iter = 1:Niterations


    rho_mid = ...
        (rho_low + rho_high)/2;


    [q_test,success] = ...
        solveVelocityConstrainedProblem( ...
            rho_mid, ...
            q_feasible, ...
            H, ...
            Aeq, ...
            beq, ...
            E, ...
            n, ...
            t_constraint, ...
            optionsFMINCON ...
        );


    %% --------------------------------------------------------
    % Verify solution using a finer grid
    % ---------------------------------------------------------

    if success


        P_test = [ ...
            q_test(1:Ncp)';
            q_test(Ncp+1:end)' ...
        ];


        max_speed_test = ...
            maximumSpeed( ...
                P_test, ...
                n, ...
                t_verify ...
            );


        waypoint_error = ...
            norm( ...
                Aeq*q_test ...
                - beq, ...
                inf ...
            );


        feasible_verified = ...
            max_speed_test <= rho_mid + 1e-5 ...
            && ...
            waypoint_error <= 1e-6;


    else

        feasible_verified = false;

    end


    %% --------------------------------------------------------
    % Binary-search update
    % ---------------------------------------------------------

    if feasible_verified


        rho_high = rho_mid;

        q_feasible = q_test;


    else


        rho_low = rho_mid;


    end


end


%% ============================================================
% 15. rho_min
% ============================================================

rho_min = rho_high;


%% Final constrained solution

[q_min,success] = ...
    solveVelocityConstrainedProblem( ...
        rho_min, ...
        q_feasible, ...
        H, ...
        Aeq, ...
        beq, ...
        E, ...
        n, ...
        t_constraint, ...
        optionsFMINCON ...
    );


if ~success

    error('Final rho_min optimization failed.');

end


P_min = [ ...
    q_min(1:Ncp)';
    q_min(Ncp+1:end)' ...
];


rho_verified = ...
    maximumSpeed( ...
        P_min, ...
        n, ...
        t_verify ...
    );


%% ============================================================
% 16. RESULTS
% ============================================================

fprintf('\n');
fprintf('============================================\n');
fprintf('FINAL RESULTS\n');
fprintf('============================================\n');


fprintf( ...
    'Lower theoretical bound = %.8f\n', ...
    rho_lower_bound ...
);


fprintf( ...
    'rho_min                 = %.8f\n', ...
    rho_min ...
);


fprintf( ...
    'Verified maximum speed  = %.8f\n', ...
    rho_verified ...
);


fprintf( ...
    'rho_free                = %.8f\n', ...
    rho_free ...
);


fprintf('\n');

fprintf( ...
    'Feasibility interval:\n' ...
);

fprintf( ...
    'rho >= %.8f\n', ...
    rho_min ...
);


fprintf('\n');

fprintf( ...
    'Interval where the velocity constraint can affect the optimum:\n' ...
);

fprintf( ...
    '[%.8f , %.8f]\n', ...
    rho_min, ...
    rho_free ...
);


%% ============================================================
% 17. EVALUATE TRAJECTORIES
% ============================================================

Z_free = evaluateBezier( ...
    P_free, ...
    n, ...
    t_verify ...
);


Z_min = evaluateBezier( ...
    P_min, ...
    n, ...
    t_verify ...
);


speed_min = zeros( ...
    1, ...
    Nverify ...
);


for k = 1:Nverify


    zd = bezierVelocity( ...
        P_min, ...
        n, ...
        t_verify(k) ...
    );


    speed_min(k) = norm(zd);

end


%% ============================================================
% 18. PLOT TRAJECTORIES
% ============================================================

figure;

hold on;
grid on;
axis equal;


plot( ...
    Z_free(1,:), ...
    Z_free(2,:), ...
    'LineWidth',2, ...
    'DisplayName','Unconstrained' ...
);


plot( ...
    Z_min(1,:), ...
    Z_min(2,:), ...
    'LineWidth',2, ...
    'DisplayName','Velocity constrained' ...
);


plot( ...
    W(:,1), ...
    W(:,2), ...
    'ks', ...
    'MarkerFaceColor','k', ...
    'MarkerSize',8, ...
    'DisplayName','Waypoints' ...
);


xlabel('x');

ylabel('y');


title( ...
    'Dubins trajectory with velocity constraint' ...
);


legend('Location','best');


%% ============================================================
% 19. PLOT VELOCITY
% ============================================================

figure;

hold on;
grid on;


plot( ...
    t_verify, ...
    speed_min, ...
    'LineWidth',2, ...
    'DisplayName','||v(t)||' ...
);


yline( ...
    rho_min, ...
    'r--', ...
    '\rho_{min}', ...
    'LineWidth',1.5 ...
);


xlabel('Normalized time');

ylabel('Velocity norm');


title( ...
    'Velocity constraint at \rho_{min}' ...
);


legend('Location','best');


%% ============================================================
% 20. COMPARE UNCONSTRAINED / CONSTRAINED VELOCITY
% ============================================================

figure;

hold on;
grid on;


plot( ...
    t_verify, ...
    speed_free, ...
    'LineWidth',2, ...
    'DisplayName','Unconstrained' ...
);


plot( ...
    t_verify, ...
    speed_min, ...
    'LineWidth',2, ...
    'DisplayName','Constrained' ...
);


yline( ...
    rho_min, ...
    'k--', ...
    '\rho_{min}' ...
);


xlabel('Normalized time');

ylabel('Velocity norm');


title( ...
    'Unconstrained vs constrained velocity' ...
);


legend('Location','best');


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
% Bezier velocity
% ============================================================

function zd = bezierVelocity(P,n,t)


    zd = zeros(2,1);


    for i = 0:n-1

        zd = ...
            zd ...
            + n ...
            * ( ...
                P(:,i+2) ...
                - P(:,i+1) ...
              ) ...
            * bernstein( ...
                i, ...
                n-1, ...
                t ...
              );

    end

end


%% ============================================================
% Maximum velocity
% ============================================================

function vmax = maximumSpeed(P,n,t)


    speed = zeros( ...
        1, ...
        length(t) ...
    );


    for k = 1:length(t)


        zd = bezierVelocity( ...
            P, ...
            n, ...
            t(k) ...
        );


        speed(k) = norm(zd);

    end


    vmax = max(speed);

end


%% ============================================================
% Evaluate Bezier trajectory
% ============================================================

function Z = evaluateBezier(P,n,t)


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
% Solve velocity-constrained problem
% ============================================================

function [q_opt,success] = ...
    solveVelocityConstrainedProblem( ...
        rho, ...
        q0, ...
        H, ...
        Aeq, ...
        beq, ...
        E, ...
        n, ...
        t_constraint, ...
        options ...
    )


    objective = @(q) ...
        q' * H * q;


    nonlcon = @(q) ...
        velocityConstraints( ...
            q, ...
            rho, ...
            E, ...
            n, ...
            t_constraint ...
        );


    [q_opt,~,exitflag] = fmincon( ...
        objective, ...
        q0, ...
        [],[], ...
        Aeq, ...
        beq, ...
        [],[], ...
        nonlcon, ...
        options ...
    );


    success = exitflag > 0;

end


%% ============================================================
% Velocity constraints
%
% fmincon requires:
%
%       c(q) <= 0
%
% therefore:
%
%       ||z_dot(t)||^2 - rho^2 <= 0
% ============================================================

function [c,ceq] = velocityConstraints( ...
    q, ...
    rho, ...
    E, ...
    n, ...
    t_constraint ...
)


    Ncp = n + 1;


    Px = q(1:Ncp);

    Py = q(Ncp+1:end);


    dPx = E*Px;

    dPy = E*Py;


    c = zeros( ...
        length(t_constraint), ...
        1 ...
    );


    for k = 1:length(t_constraint)


        t = t_constraint(k);


        Bdot = zeros(n,1);


        for i = 0:n-1

            Bdot(i+1) = ...
                bernstein( ...
                    i, ...
                    n-1, ...
                    t ...
                );

        end


        xdot = ...
            n ...
            * Bdot' ...
            * dPx;


        ydot = ...
            n ...
            * Bdot' ...
            * dPy;


        c(k) = ...
            xdot^2 ...
            + ydot^2 ...
            - rho^2;

    end


    ceq = [];

end