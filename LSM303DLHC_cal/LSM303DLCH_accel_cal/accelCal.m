% This file loads the data captured during execution of LSM303DLCH_accel_cal.ino and calculates
% the appropriate calibration constants for the device.

% Load data
fprintf('Loading data ...\n');

data = load('accelData.txt');

% parse data into matrices Y and W
Y = data(:,1:3);
W = data(:,4:7);

fprintf('Calculating calibration constants ...\n');
% calculate calibration constants via least squares
X = (W' * W)^-1 * W' * Y;

fprintf('Copy this into your code that calibrates the accelerometer outputs: \n')
fprintf('float acc11 = %f;\n',X(1,1));
fprintf('float acc12 = %f;\n',X(2,1));
fprintf('float acc13 = %f;\n',X(3,1));
fprintf('float acc10 = %f;\n',X(4,1));

fprintf('float acc21 = %f;\n',X(1,2));
fprintf('float acc22 = %f;\n',X(2,2));
fprintf('float acc23 = %f;\n',X(3,2));
fprintf('float acc20 = %f;\n',X(4,2));

fprintf('float acc31 = %f;\n',X(1,3));
fprintf('float acc32 = %f;\n',X(2,3));
fprintf('float acc33 = %f;\n',X(3,3));
fprintf('float acc30 = %f;\n',X(4,3));
fprintf('end copy ... \n')

% fprintf('results ...\n');
% for i = 1:length(W)
%     fprintf('uncal error: %10.2e %10.2e %10.2e cal error: %10.2e %10.2e %10.2e \n', abs(Y(i,:) - W(i,1:3)),abs(Y(i,:) - W(i,:)*X));
% endfor





