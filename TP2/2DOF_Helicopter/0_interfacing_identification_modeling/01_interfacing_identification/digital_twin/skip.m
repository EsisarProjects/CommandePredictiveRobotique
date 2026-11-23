% Identification 
% Fortunately, there exists a built-in matlab function (tfest for "transfer function estimation") that determines a transfer function from our recorded time series data. We need to fix the order of the denominator and denominator of the transfer function. These two numbers read "2" and "0" for the order of the denominator and numerator, respectively, because the polynomial in the denominator and numerator of our target transfer function
%% Load measured data from past run
load('DataPitchStep.mat');
% store in variables
t = DataPitchStep(1,:); % time (s)
Vp = DataPitchStep(2,:); % pitch voltage (V)
wm_p = DataPitchStep(3,:); % rotor speed (rad/s)
theta = DataPitchStep(4,:); % pitch angle (rad)
delta_t = 0.04; 
identData= iddata(theta(40000:20:57500)', Vp(40000:20:57500)', delta_t); 
transferFunction= tfest(identData, 2, 0);  
denom = transferFunction.Denominator;
num = transferFunction.Numerator;
Ksp= Jp * denom(3);
Dp = Jp * denom(2);
Kpp = Jp * num * Dt;
% Display identified parameters
disp(['Ksp: ', num2str(Ksp)]);
disp(['Dp: ', num2str(Dp)]);
disp(['Kpp_id: ', num2str(Kpp)]);