clear all;close all;clc;tic
%% Parameters
shape = "legendre";  % poly = polynomial, cheby = chebyshev, legendre = legendre
Nmode = 2;           % # shape functions to approximate strain/curvature

L = 1;               % undeformed length of actuator
q = [0;0;0.1;0];    % q(t) = q(0)

%% Contrained strain/curvature, 0 = contrained 1 = free
K1 = 0;  % curvatures
K2 = 1;
K3 = 0;

E1 = 0;  % strains
E2 = 0;
E3 = 1;

K = [K1;K2;K3];
E = [E1;E2;E3];
xi_ac = [K;E];   % xi with a(ctive) and c(onctrained) strains

%% Create actuation matrix Ba and its complementary Bc

n  = length(find(xi_ac == 1));              % # active curvatures/strains
Ba = zeros(length(xi_ac),n);                % pre-alociation
Bc = zeros(length(xi_ac),length(xi_ac)-n);  % pre-alociation

a = find(xi_ac == 1);                       % inidices active DOF
c = find(xi_ac == 0);                       % inidices contrained DOF
for ii = 1:length(a)
    Ba(a(ii),ii) =1;                        % active DOF
end
for ii = 1:length(c)
    Bc(c(ii),ii) =1;                        % complementary matrix
end

if Nmode*n ~= length(q)                     % throw error when q is not of satisfactory length
    error('q should have length Nmode*active strains')
end

%% Forward kinematics

g0 = [1;0;0;0;0;1;0;0;0;0;1;0;0;0;0;1];   % initial condition g0=eye(4,4)
[l, g] = ode45(@(l,y) forwardKinematics(l,y,q,Ba,shape,Nmode),[0 L],g0); % solve forward kinematics

%% Present output in SE3
Rot = cell(1,length(l));    

for ii = 1:length(l)
Rot{ii}     = [g(ii,1),g(ii,2),g(ii,3);     % robot's rotation 
               g(ii,5),g(ii,6),g(ii,7);
               g(ii,9),g(ii,10),g(ii,11)]; 
end

xdata = g(:,4);                       % robot's translation x
ydata = g(:,8);                       % robot's translation y
zdata = g(:,12);                       % robot's translation z 

toc
[actuator_length,~] = arclength(xdata,ydata,zdata,'s')

figure(1)
subplot(1,2,1)
plot3(xdata,ydata,-zdata)
xlabel('x'); ylabel('y');zlabel('z')
grid on; box on;
subplot(1,2,2)
hold on
plot(xdata,-zdata)
grid on;box on
xlabel('x');ylabel('z')

toc


