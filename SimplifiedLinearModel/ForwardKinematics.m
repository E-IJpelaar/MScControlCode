clear all;close all;clc;
%% 

l = 1; % length of actuator
t = 10;% time

sdisc = 1000;
tdisc = 1000;

ds = l/sdisc;
ts = 1/tdisc;

l = linspace(0,l,l/ds);

nmode = 1; % only one shape function to approximate
ndisc = 1; % single link


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

% q0 = [0;0];     % for constant q(t) and only 1 shape function
% [t,eta] = ode45(@(t,eta) strainField(t,eta,s), [0 t],q0);


%% Compute strainfield
for ii = 1:length(l)
    
    % q(t) = constant
    % shape function = chebyshev to degree 3
    q = [1;0.5;0.1;1;0.5;0.1];
    s = l(ii);
    Phi = [ 1,s,2*s^2-1,0,0,0;
            0,0,0,1,s,2*s^2-1];
    xi(:,ii) = Phi*q;   % strain at eacht point in sigma
  
end

%% compute forward kinematics


g0 = eye(4);
g0 = reshape(g0,[],1);

[l,g] = ode45(@(l,g) forwardIntegration(l,g,xi), [0,l],g0);


for ii = 1:length(l)
G{ii} = reshape(g(ii,:),[4,4]);
xdata(ii) = G{ii}(1,4);
ydata(ii) = G{ii}(2,4);
zdata(ii) = G{ii}(3,4);
end

[arclen,seglen] = arclength(xdata,ydata,zdata,'l');

figure(1)

plot3(xdata,ydata,zdata)
xlabel('x'); ylabel('y');zlabel('z')
grid on; box on;

% g0 = eye(4);
% g0 = reshape(g0,[],1);
% [l,y] = ode45(@(t,y) forwardKin(t,y),[0 l],g0);
% 




         