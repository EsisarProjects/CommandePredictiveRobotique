%%
clear; clc; close all;

%%
% Load measured data from past run
load('DataPPControl.mat');
% store in variables
t = DataPPControl(1,:); % time (s)
Vp = DataPPControl(2,:); % pitch voltage (V)
theta_d = 180/pi*DataPPControl(3,:); % desired pitch angle (deg)
theta = 180/pi*DataPPControl(4,:); % pitch angle (deg)
Vy = DataPPControl(5,:); % yaw voltage (V)
yaw_d = 180/pi*DataPPControl(6,:); % desired yaw angle (deg)
yaw = 180/pi*DataPPControl(7,:); % yaw angle (deg)
%
figure;
subplot(2,2,1);
plot(t,theta_d,t,theta);
title('Pitch');
ylabel('Angle (deg)');
subplot(2,2,3);
plot(t,Vp);
ylabel('Pitch Motor (V)');
xlabel('time (s)');
% 
subplot(2,2,2);
plot(t,yaw_d,t,yaw);
title('Yaw');
ylabel('Angle (deg)');
subplot(2,2,4);
plot(t,Vy);
ylabel('Yaw Motor (V)');
xlabel('time (s)');
sgtitle('Pole placement PD Control Response');
