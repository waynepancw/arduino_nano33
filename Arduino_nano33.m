clear all, close all, clc

a = arduino;
lsmObj = lsm9ds1(a,"Bus",1)

%% read data

acc = readAcceleration(lsmObj)
angV = readAngularVelocity(lsmObj)
magF = readMagneticField(lsmObj)

%% analysis imu data
% open_system('AnalyseIMUData.slx');

