# 9-dof-sensor-calibration
This repo contains utilities to calibrate the LSM303 and L3GD20H sensors (See Adafruit 9-dof and 10-dof breakouts).

It makes use of and I2C library and specific libraries for the sensors (link to repos).  It produces data formated for use by a matlab/octave utility that post processes the sensor data to provide the calculated calibration constants.
