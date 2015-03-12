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

X

