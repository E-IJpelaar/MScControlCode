clear all;close all;clc;

data = load ('PressurePID.txt');


t = data(:,1);
p = data(:,4);
p_ref = data(:,7);
u = data(:,5);


figure(1)
subplot(2,1,1)
plot(t,p)
hold on; grid on;
plot(t,p_ref)
xlabel('Time [s]');ylabel('Pressure [hPa]')


subplot(2,1,2)
plot(t,u)
hold on; grid on;
xlabel('Time [s]');ylabel('Input [-]')