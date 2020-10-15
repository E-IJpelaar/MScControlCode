clear all;close all;clc;
%% Parameters

L = 1; % length of actuator
t = 10;% time


%% Contrained strain/curvature 0 = contrained 1 = free

E1 = 0;   
E2 = 0;
E3 = 1;
K1 = 1;
K2 = 0;
K3 = 0;

K = [K1;K2;K3];
E = [E1;E2;E3];
eta = [K;E];

%% Create Ba matrix

n = length(find(eta == 1));
Ba = zeros(length(eta),n);
Bc = zeros(length(eta),length(eta)-n);

a = find(eta == 1);
c = find(eta == 0);
for ii = 1:length(a)
    Ba(a(ii),ii) =1;
end
for ii = 1:length(c)
    Bc(c(ii),ii) =1;
end

% Bb = zeros(length(eta),length(eta));
% for ii = 1:length(eta)
%     if eta(ii) ==1;
%         Bb(ii,ii) =1;       
%     end
% end


%% Compute strainfield

% q = [1;0.5;0.1;1;0.5;0.1];
% syms s
% phi = chebyshevT(3,s);
% xi = cross(Bb,phi)*q;
% 
% shape_order = 2;
% 
% [rowBa,colBa] = size(Ba);
% phi = zeros(colBa,shape_order*colBa);
% sym s


%% Assume known strainfield
% fully actuated q = [1;1], independent of t
% Phi = [1,0;0,1]
q = [0.5;0.5];
Phi = eye(2);
xi = Phi*q; % for all t







%% Forward kinematics

g0 = eye(4);
g0 = reshape(g0,[4,4]);

[l,g] = ode45(@(l,g) forwardIntegration(l,g,xi),[0 L],g0);


%% Present output in SE3

for ii = 1:length(l)
G{ii} = reshape(g(ii,:),[4,4]);
xdata(ii) = G{ii}(1,4);
ydata(ii) = G{ii}(2,4);
zdata(ii) = G{ii}(3,4);
end


[arclen,~] = arclength(xdata,ydata,zdata,'l');

figure(1)

plot3(xdata,ydata,zdata)
xlabel('x'); ylabel('y');zlabel('z')
grid on; box on;








