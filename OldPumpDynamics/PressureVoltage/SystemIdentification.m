clear all;close all;clc;
addpath('C:\Users\s159261\OneDrive - TU Eindhoven\Documents\GitHub\MScControlCode\PumpDynamics\PressureVoltage\Exp1102OudeActuator')
hPa2kPa = 0.1;

V_input = 10;
name = (['data', num2str(V_input),'.txt']);
data = load (name);


t = data(:,1);
dp = data(:,3);
pwm = data(:,5);
