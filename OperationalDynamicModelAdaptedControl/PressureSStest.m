clear all;close all;clc;

V = 2;
tau = 0.095;

[t,p] = ode45(@(t,p) pressureModel(t,p,V,tau),[0 1],0);

figure(1)
plot(t,p)