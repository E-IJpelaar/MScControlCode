clear all;close all;clc;


a1 = 2e3;
a2 = 1e3;
a3 = -4.55e1;
epsilon = linspace(-0.015,0.02,1000);


k = a1 + a2.*((tanh((a3.*epsilon).^2))-1);
F = 0.5*k.*epsilon;

figure(1)
plot(epsilon,F)
axis([-0.015 0.02 -15 20])
