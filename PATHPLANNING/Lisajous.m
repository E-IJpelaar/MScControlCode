clear all;close all;clc;




A = 0.014;
a = 0.3;

B = -0.007;
b = 0.6
delta = 0.68;

y_offset = -0.072+B;
t0 = 34/15;

t  = linspace(0,(2*pi)/a,((2*pi)/a)/0.0556);

x  = A*sin(a.*(t-t0) + delta);
y  = y_offset + B*sin(b.*(t-t0));

figure(1)
plot(x,y,'o')
hold on;grid on
plot(x(1),y(1),'rx','MarkerSize',12,'LineWidth',2)
text(x(1),y(1),'t=0')
plot(x(length(t)/2),y(length(t)/2),'rx','MarkerSize',12,'LineWidth',2)
text(x(length(t)/2),y(length(t)/2),['t= ',num2str(t(end)/2)])
plot(x(3*length(t)/4),y(3*length(t)/4),'rx','MarkerSize',12,'LineWidth',2)
text(x(3*length(t)/4),y(3*length(t)/4),['t= ',num2str(3*t(end)/4)])
plot(x(length(t)/4),y(length(t)/4),'rx','MarkerSize',12,'LineWidth',2)
text(x(length(t)/4),y(length(t)/4),['t= ',num2str(t(end)/4)])
xlabel('X')
ylabel('Y')


figure(2)
plot(t,x)
hold on;grid on;
plot(t,y)
legend('x','y')
xlabel('Time')
ylabel('Value [m]')