% Find moment of inertia of pendulum

%%
clear; clc; close all;

% import parameters of Qube Servo 2
qube2_rotpen_param;
%
delta_t = 12.378;    % total duration seconnds
n_cycle = 20;        % number of cycles
fn = n_cycle/delta_t;

% Natural frequency (Hz)

% wn = 0;
wn = 2*pi*fn;

% Moment of inertia - experimental (kg-m^2)
% Jp_exp = 0;
Jp_exp = mp * g * l / wn^2;

% Moment of inertia - calculated (kg-m^2) [note: COF about pivot]
% Jp = 0;
Jp = mp*Lp^2/3;

%%
fprintf("Moment of inertia - experimental (kg-m^2): Jp_exp=%f\n",Jp_exp);
fprintf("Moment of inertia - calculated (kg-m^2): Jp=%f\n",Jp);