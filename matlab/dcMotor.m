clear all;
close all;
clc;

Ts = 0.01;
sampleSize = 668;
Tsim = Ts * sampleSize;
t_vec = [0:Ts:Tsim];

RPM = csvread('prbs_values.csv', 1, 0, [1, 0, 668, 0]);
PWM = csvread('prbs_values.csv', 1, 1);

averageRPM = mean(RPM);
deltaAverage = RPM - averageRPM;
pwmAverage = mean(PWM);
deltaPWM = PWM - pwmAverage;

modelData = iddata(deltaAverage, deltaPWM, Ts);
%sys = arx(dat, [3 3 0]); % 84.77% acc
plot(modelData)

%sys = armax(dat, [3 2 2 1]) % 83.59% acc
sys = bj(modelData, [3 2 1 3 1]) %88.24% acc
figure(1)
compare(modelData, sys)
title("Filtered PRBS signal")