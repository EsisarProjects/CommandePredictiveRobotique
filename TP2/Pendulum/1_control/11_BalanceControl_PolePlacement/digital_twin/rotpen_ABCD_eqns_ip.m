% Find Total Inertia
Jt = Jr*Jp - mp^2*r^2*l^2;
% 
% State Space Representation
A = ...;
B = ...;
C = ...;
D = ...;

% 
% Add actuator dynamics
A(3,3) = A(3,3) - kt*km/Rm*B(3);
A(4,3) = A(4,3) - kt*km/Rm*B(4);
B = kt * B / Rm;