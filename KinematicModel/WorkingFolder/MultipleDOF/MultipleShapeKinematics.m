clear all;close all;clc;
%% Parameters

L = 1; % length of actuator

q = [0.1;0];



%% Contrained strain/curvature 0 = contrained 1 = free

E1 = 0;   
E2 = 0;
E3 = 1;
K1 = 0;
K2 = 1;
K3 = 0;

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
    Ba(a(ii),ii) =1;
end
for ii = 1:length(c)
    Bc(c(ii),ii) =1;
end


%% 
g0 = [1,0,0,0;0,1,0,0;0,0,1,0;0,0,0,1]; 
[l,g] = ode45(@(l,g) forwardIntegration(l,g,q,Ba),[0,L],g0);



% boyer forward dynamics, quadternions

%% Present output in SE3

for ii = 1:length(l)
G{ii} = reshape(g(ii,:),4,4);
xdata(ii) = G{ii}(1,4);
ydata(ii) = G{ii}(2,4);
zdata(ii) = G{ii}(3,4);
end

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







