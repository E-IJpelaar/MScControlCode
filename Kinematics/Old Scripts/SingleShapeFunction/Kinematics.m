clear all;close all;clc;
%% Parameters

L = 1; % length of actuator

q = [0;0]; %q(t) = q(0)

%% Contrained strain/curvature 0 = contrained 1 = free

K1 = 0;  % curvatures
K2 = 1;
K3 = 0;

E1 = 0;  % strains
E2 = 0;
E3 = 1;

K = [K1;K2;K3];
E = [E1;E2;E3];
xi = [K;E];

%% Create Ba matrix

n = length(find(xi == 1));
Ba = zeros(length(xi),n);
Bc = zeros(length(xi),length(xi)-n);

a = find(xi == 1);
c = find(xi == 0);
for ii = 1:length(a)
    Ba(a(ii),ii) =1;  % active DOF
end
for ii = 1:length(c)
    Bc(c(ii),ii) =1;  % complementary matrix
end

g0 = [1;0;0;0;0;1;0;0;0;0;1;0;0;0;0;1];
[l, g] = ode45(@(l,y) forwardKinematics(l,y,q,Ba),[0 L],g0);

%% Present output in SE3
Rot = cell(1,length(l));
xdata = zeros(1,length(l));
ydata = zeros(1,length(l));
zdata = zeros(1,length(l));
for ii = 1:length(l)
Rot{ii}     = [g(ii,1),g(ii,5),g(ii,9);
               g(ii,2),g(ii,6),g(ii,10);
               g(ii,3),g(ii,7),g(ii,11)]; 
xdata(ii) = g(ii,13);
ydata(ii) = g(ii,14);
zdata(ii) = g(ii,15);
end


[arc,~] = arclength(xdata,ydata,zdata,'s');

figure(1)
subplot(1,2,1)
plot3(xdata,ydata,-zdata)
xlabel('x'); ylabel('y');zlabel('z')
grid on; box on;
subplot(1,2,2)
plot(xdata,-zdata)
grid on;box on
xlabel('x');ylabel('z')
%





