%% Exercise 2.4 - 3D Swapping Cube (8 UAVs sem colisão)
clear; clc; close all;

% 1. General parameters
T  = 12;          % Total maneuver time [s]
h  = 0.05;        % Time step [s]
tt = 0:h:T;
N  = length(tt);
L  = 50;          % Half-width of the cube (100x100x100 m)

% 2. Definition of the 8 initial and final vertices
V_init = [ -L, -L, -L;
            L, -L, -L;
            L,  L, -L;
           -L,  L, -L;
           -L, -L,  L;
            L, -L,  L;
            L,  L,  L;
           -L,  L,  L ];
V_final = -V_init;

% 3. Matrix M 6x6 (polynomial of degree 5)
M = [ 1,  0,    0,      0,       0,       0;
      0,  1,    0,      0,       0,       0;
      0,  0,    2,      0,       0,       0;
      1,  T,  T^2,    T^3,     T^4,     T^5;
      0,  1,  2*T,  3*T^2,   4*T^3,   5*T^4;
      0,  0,    2,    6*T,  12*T^2,  20*T^3 ];

traj = zeros(8, 3, N);
Va_init = 15;

% 4. Generation of trajectories with asymmetric anti-collision deviation
for k = 1:8
    p0 = V_init(k, :);
    pf = V_final(k, :);
    
    % Lower layer (k=1..4) rotates counterclockwise; upper layer (k=5..8) clockwise
    dir_giro = sign(p0(3)); 
    ang_radial = atan2(p0(2), p0(1));
    
    % Asymmetrical tangential initial velocity (avoids passing through the origin at t = T/2)
    vx_0 = -dir_giro * Va_init * sin(ang_radial);
    vy_0 =  dir_giro * Va_init * cos(ang_radial);
    
    % Final speed different from initial velocity to break the odd symmetry.
    vx_f =  dir_giro * Va_init * sin(ang_radial);
    vy_f = -dir_giro * Va_init * cos(ang_radial);
    
    % Initial radial acceleration outward (pushes the trajectory away from the center)
    ax_0 =  0.25 * p0(1);
    ay_0 =  0.25 * p0(2);
    az_0 =  0.15 * p0(3);
    
    bx = [p0(1); vx_0; ax_0; pf(1); vx_f; 0];
    by = [p0(2); vy_0; ay_0; pf(2); vy_f; 0];
    bz = [p0(3); 0;    az_0; pf(3); 0;    0];
    
    ax = M \ bx;
    ay = M \ by;
    az = M \ bz;
    
    T_mat = [ones(1,N); tt; tt.^2; tt.^3; tt.^4; tt.^5];
    traj(k, 1, :) = ax' * T_mat;
    traj(k, 2, :) = ay' * T_mat;
    traj(k, 3, :) = az' * T_mat;
end

% 5. Safety check (Minimum distance between any 2 drones)
min_dist = inf;
for t_idx = 1:N
    pts = traj(:, :, t_idx);
    for i = 1:7
        for j = i+1:8
            d = norm(pts(i,:) - pts(j,:));
            if d < min_dist
                min_dist = d;
            end
        end
    end
end
fprintf('Distância mínima entre quaisquer dois drones: %.2f m\n', min_dist);

% 6. 3D Plotting
figure('Color', 'w');
hold on; grid on; view(3);
colors = lines(8);

for k = 1:8
    x_k = squeeze(traj(k, 1, :));
    y_k = squeeze(traj(k, 2, :));
    z_k = squeeze(traj(k, 3, :));
    
    plot3(x_k, y_k, z_k, 'Color', colors(k,:), 'LineWidth', 2);
    plot3(x_k(1), y_k(1), z_k(1), 'o', 'Color', colors(k,:), 'MarkerFaceColor', colors(k,:), 'MarkerSize', 7);
    plot3(x_k(end), y_k(end), z_k(end), 's', 'Color', colors(k,:), 'MarkerSize', 9, 'LineWidth', 1.5);
end

xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
title(sprintf('Swapping Cube 3D - 8 UAVs (Minimum distance: %.2f m)', min_dist));
axis equal;