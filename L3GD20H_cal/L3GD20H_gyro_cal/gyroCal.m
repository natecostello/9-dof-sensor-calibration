% This file loads the data captured during execution of L3GD20H_gyro_cal.ino and calculates
% the appropriate calibration constants for the device.

% For methodoly used to calibrate the gyros refer to:
% "A Simple Calibration For Mems Gyroscopes" by Mark Looney of Analog Devices

% Load data
fprintf('Loading data ...\n');
load -text gyroData.txt;
whos
% plot(x_u_static(:,1));
% hold on;
% plot(y_u_static(:,1));
% plot(z_u_static(:,1));
% plot(x_u_static(:,2), 'r');
% plot(y_u_static(:,2), 'r');
% plot(z_u_static(:,2), 'r');
% plot(x_u_static(:,3), 'g');
% plot(y_u_static(:,3), 'g');
% plot(z_u_static(:,3), 'g');
% pause();

xbias = mean([x_u_static(:,1); y_u_static(:,1); z_u_static(:,1)]);
ybias = mean([x_u_static(:,2); y_u_static(:,2); z_u_static(:,2)]);
zbias = mean([x_u_static(:,3); y_u_static(:,3); z_u_static(:,3)]);
bias = [xbias ybias zbias];

%remove bias from dynamic runs
x_u_ccw = x_u_ccw - bias;
y_u_ccw = y_u_ccw - bias;
z_u_ccw = z_u_ccw - bias;
x_u_cw = x_u_cw - bias;
y_u_cw = y_u_cw - bias;
z_u_cw = z_u_cw - bias;

% note with a conversion of 0.0175 degrees per LSB:
% 180 degrees equals 10286;

% x scale factor
t = 0:delta_t:(length(x_u_ccw)-1)*delta_t;
t = t';
z = 0:delta_t:(length(x_u_cw)-1)*delta_t;
z = z';
p = trapz(x_u_cw(:,1), z);
q = trapz(x_u_ccw(:,1), t);
x_scale = (abs(10286/p) + abs(10286/q))/2;

% y scale factor
t = 0:delta_t:(length(y_u_ccw)-1)*delta_t;
t = t';
z = 0:delta_t:(length(y_u_cw)-1)*delta_t;
z = z';
p = trapz(y_u_cw(:,2), z);
q = trapz(y_u_ccw(:,2), t);
y_scale = (abs(10286/p) + abs(10286/q))/2;

% z scale factor
t = 0:delta_t:(length(z_u_ccw)-1)*delta_t;
t = t';
z = 0:delta_t:(length(z_u_cw)-1)*delta_t;
z = z';
p = trapz(z_u_cw(:,3), z);
q = trapz(z_u_ccw(:,3), t);
z_scale = (abs(10286/p) + abs(10286/q))/2;

fprintf('Copy this into your code that calibrates the accelerometer outputs: \n')
fprintf('float gcc11 = %f;\n',x_scale);
fprintf('float gcc12 = %f;\n',0);
fprintf('float gcc13 = %f;\n',0);
fprintf('float gcc10 = %f;\n',xbias);

fprintf('float gcc21 = %f;\n',0);
fprintf('float gcc22 = %f;\n',y_scale);
fprintf('float gcc23 = %f;\n',0);
fprintf('float gcc20 = %f;\n',ybias);

fprintf('float gcc31 = %f;\n',0);
fprintf('float gcc32 = %f;\n',0);
fprintf('float gcc33 = %f;\n',z_scale);
fprintf('float gcc30 = %f;\n',zbias);
fprintf('end copy ... \n')

