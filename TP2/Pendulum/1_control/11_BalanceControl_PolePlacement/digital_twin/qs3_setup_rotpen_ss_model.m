%% 
clear; clc; close all;

%% calculate state-space model matrices

% Load QUBE-Servo 3 model parameters
qs3_rotpen_param;
% Load state-space model for pendulum in upright configuration
rotpen_ABCD_eqns_ip;
% Create Matlab state-space object for analysis
sys = ss(A,B,C,D);

%% open-loop poles of inverted pendulum
% Hint: eigenvalues of matrix A: eig(A) 
% poles_open = ...;

% Display the open-loop poles
disp('Open-loop poles of the inverted pendulum:');
disp(poles_open);
if all(poles_open<0)
    disp('The system is stable.');
else
    disp('The system is unstable.');
end

%% controllability and observability
% Hint: using ctrb() and obsv() function in MATLAB
% Co = ctrb(...);
% Ob = obsv(...);


% Rank of controllability and observability matrices
disp('Rank of controllability matrix:');
disp(rank(Co));
disp('Rank of observability matrix:');
disp(rank(Ob));

% Check controllability and observability
if rank(Co) == size(A, 1)
    disp('The system is controllable.');
else
    disp('The system is not controllable.');
end

% Check observability
if rank(Ob) == size(A, 1)
    disp('The system is observable.');
else
    disp('The system is not observable.');
end

%% Control synthesis: Pole placement
% Find dominant poles corresponding to specifications:
% PO < 6.81%
% t_settling < 1.54s


% - Transfer function:
% G(s) = Y(s)/U(s) = wn^2 / ( s^2 + 2*zeta*wn*s + wn^2 )
% - In order form: 
% G(s) = Y(s)/U(s) = (sigma^2 + wd^2) / ( s^2 - 2*sigma*s + (sigma^2 + wd^2) )
%      = (sigma^2 + wd^2) / ( (s - sigma + j*wd) * (s - sigma - j*wd) )

% Poles are at: s = sigma +/- j*wd  
% where:
% - wn = sqrt(sigma^2 + wd^2) : natural frequency
% - wd = wn * sqrt(1 - zeta^2): damped frequency
% - sigma = - zeta * wn : decay rate
% - zeta = |sigma| / wn : damping ratio

% zeta > 1      : overdamped         : 2 distinct real roots
% zeta = 1      : critically damped  : 2 equal real roots
% 0 < zeta < 1  : underdamped        : 2 complex conjugate roots

% Recall step response for complex poles:
% Y(s) = wn^2 / ( s^2 + 2*zeta*wn*s + wn^2 ) * U(s) where U(s) = 1/s % step
% => y(t) = 1 - e^(sigma * t)( cos(wd*t) + sigma / wd * sin(wd * t) )
% <=> y(t) = 1 - e^(sigma * t) * (wn/wd) * sin(wd * t + phi)
% where: phi = arctan(wd / (zeta * wn) )

% Peak time: Time at which step response reaches maximum
% - y'(t=Tp) = 0 = (wn^2 / wd) * e^(sigma * t) * sin(wd * t)
% => t = Tp = n * pi / wd. So the first peak is the maximum(n=1)
% => Tp = pi / wd = pi / ( wn * sqrt(1 - zeta^2) )

% Percentage Overshoot: peak value of the signal as percentage of steady-state
% PO = 100% *  y(Tp) - 1 % step response y_ss = 1
% PO = 100% * exp( sigma * Tp ) = 100% * exp( sigma * pi / wd )
%    = 100% * exp( - pi * zeta / sqrt(1 - zeta^2) )
% ==> PO depends only on zeta
% PO < PO_max
% => sigma * pi / wd > log(PO_max) % since PO_max < 1
% or wd < sigma * pi / log(PO_max)

% Calculate damping ratio zeta from PO: 
% zeta = -log(PO/100) / sqrt( pi^2 + log^2(PO/100))

% Settling time: Oscillations are confined within an Exponential Envelope
% The time required for the response curve to reach and stay within 
% a range of certain percentage (usually 5% or 2%) of the final value.
% For 5%: t_s = 3 / (wn * zeta)
% For 2%: t_s = 4 / (wn * zeta) or t_s = 4.6 / (-sigma)
% => t_s < t_s_max => sigma < -4.6 / t_s_max
% => The real part need to be sufficiently negative

% 
PO_max = 6.81;  % percentage
t_s_max = 1.54; % seconds
PO = 6.5;       % chosen overshoot    
t_s = 1.5;      % chosen settling time
zeta = -log(PO/100) / sqrt( pi^2 + log(PO/100)^2);
wn = 4 / (t_s * zeta);

% display natural frequency and damping ratio:
disp(['Natural frequency (wn): ', num2str(wn), ' (rad/s)']);
disp(['Damping ratio (zeta): ', num2str(zeta)]) ;

% two dominant poles p1 and p2:
% sigma = ...;
% wd = ...;
sigma = - zeta * wn;
wd = wn * sqrt(1 - zeta^2);
p1 = sigma + 1i*wd; 
p2 = sigma - 1i*wd; 
% display dominant poles p1 and p2:
disp('Dominant poles of the system:');
disp([p1, p2]);

% Find the desired characteristic equation 
% if the other poles are placed at p3 = -40 and p4 = -45
p3 = -40;
p4 = -45;

% Calculate the desired characteristic equation coefficients
desired_poles = [p1, p2, p3, p4];
% format short g; % Set format to display full numbers
char_eq = poly(desired_poles);

% Display the characteristic equation
disp('Desired characteristic equation coefficients:');
disp(char_eq);

% calculate state-feedback gain K
% K = ...;

% Verify the stability of the closed-loop system with state feedback
poles_closed = eig(A - B*K);
disp('Closed-loop poles:');
disp(poles_closed);
% Display state-feedback gain K
disp('State-feedback gain K:');
disp(K);