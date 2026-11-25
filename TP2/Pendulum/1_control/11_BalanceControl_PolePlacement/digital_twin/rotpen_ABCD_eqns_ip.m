% Find Total Inertia
Jt = Jr*Jp - mp^2*l^2*r^2;

% State Space Representation (Inverted Pendulum: alpha ~ pi)
A = [0,                  0,         1,        0;
    0,                  0,         0,        1;
    0,   -mp^2*r*g*l^2/Jt, -Jp*br/Jt, -mp*r*l*bp/Jt;
    0,       Jr*mp*g*l/Jt, -mp*r*l*br/Jt, -Jr*bp/Jt];

B = [0;
    0;
    Jp / Jt;
    mp*r*l / Jt];

C = [1, 0, 0, 0;
    0, 1, 0, 0];

D = zeros(2, 1);

% Convert torque input to voltage input
A(3,3) = A(3,3) - kt*km/Rm*B(3);
A(4,3) = A(4,3) - kt*km/Rm*B(4);
B = kt * B / Rm;