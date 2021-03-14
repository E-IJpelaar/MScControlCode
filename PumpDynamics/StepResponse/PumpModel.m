clear all;close all;clc;

sample_delay = 3;
data6 = load('6V1.txt');
t6 = data6(1:end-sample_delay,1);
p6 = data6(sample_delay+1:end,3)*0.1;
u6 = data6(sample_delay+1:end,5);
u = u6(1,1);s

x0 = rand(2,1);
bestx = fminsearch(@(x) errorPressure(x,t6,p6,u6),x0);


figure(1)
plot(t6,p6)
hold on;grid on;
plot(t6,(u*bestx(1)*(1-exp(-t6/bestx(2)))))





