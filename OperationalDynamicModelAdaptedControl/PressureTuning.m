clear all;close all;clc;
global error_sum
error_sum = 0;

p_ref = [21.4725;16.7793];
p0 = [0,0];
tau_s = 3.8534;
Kp = 25;
Ki = 0;

dt = 1e-3;
T = 0:dt:5;

[t,y] = ode45(@(t,y) pControlTuning(t,y,tau_s,Kp,Ki,p_ref,dt),T,p0);



figure(1)
plot(t,y(:,1),'r')
hold on;grid on;
plot(t,y(:,2),'b')
plot(t,p_ref(1)*ones(1,length(y)),'r-')
plot(t,p_ref(2)*ones(1,length(y)),'b-')
xlabel('Time [s]'); ylabel('Pressure [kPa]')



