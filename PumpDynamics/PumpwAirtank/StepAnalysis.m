clear all;close all;clc;
addpath('C:\Users\s159261\OneDrive - TU Eindhoven\Documents\GitHub\MScControlCode\PUMPdynamics\StepResponsewTankMultipleV\LongerSteps100S')
data = load('Step.txt');

t = data(:,1);
p0 = data(:,2);
dp = data(:,3)*0.1;
p = data(:,4);
u = data(:,5);
v = u./341.3333;

figure(1)
plot(t,dp)
hold on;grid on;
xlabel('Time [s]');ylabel('Pressure difference [kPa]')