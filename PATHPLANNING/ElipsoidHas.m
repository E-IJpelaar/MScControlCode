clear all;close all;clc;

fs = 18;
t1 = 15;
T_ell = 20;

alpha = 0.015;
beta = 0.010;
L0 = -0.07;
y_off = -0.074;


t = linspace(0,t1+T_ell,(t1+T_ell)/(1/fs));
for ii = 1:length(t)

if t(ii) < t1
    x_d(ii) = 0;
    y_d(ii) = (y_off-L0)*1/t1*t(ii) + L0;
else
    x_d(ii) = alpha*cos(2*pi*t(ii)/T_ell);
    y_d(ii) = y_off-beta - beta*sin(2*pi*t(ii)/T_ell);
end

end

figure
plot(x_d,y_d,'o')
hold on;grid on;
plot(0,L0,'rx')
xlabel('x');ylabel('y')



figure
plot(t,x_d)
hold on;grid on
plot(t,y_d)
xlabel('Time [s]');ylabel('Position [m]')

