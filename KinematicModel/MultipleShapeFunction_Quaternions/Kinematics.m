clear all;close all;clc;tic
warning("quaternions are used, and are scaled to 1, therefore rotation is not correctly")
%% Parameters
shape = "legendre";   % poly = polynomial, cheby = chebyshev, legendre = legendre
Nmode = 2;            % # shape functions to approximate strain/curvature

L = 1;                % undeformed length of actuator
q = [0.1;0;0.1;0];    % q(t) = q(0)

%% Contrained strain/curvature, 0 = contrained 1 = free
K1 = 0;  % curvatures
K2 = 1;
K3 = 0;

E1 = 0;  % strains
E2 = 0;
E3 = 1;

K = [K1;K2;K3];
E = [E1;E2;E3];
xi_ac = [K;E];   % xi with a(ctive) and c(onctrained) strains/curvatures

%% Create actuation matrix Ba and its complementary Bc
n = length(find(xi_ac == 1));               % # active curvatures/strains
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
g0_rot = eye(3);             % inital rotation
g0_rot = rotm2quat(g0_rot);  % rotation to quaternion
g0_trans = zeros(3,1);       % initial translation
g0 = [g0_rot';g0_trans];     % initial condition vector

[l, g] = ode45(@(l,y) forwardKinematics(l,y,q,Ba,shape,Nmode),[0 L],g0); % solve forward kinematics

%% Interpret data
R = g(:,1:4);       % robot's rotation expressed in quaternions
x = g(:,5);         % robot's translation x
y = g(:,6);         % robot's translation y
z = g(:,7);         % robot's translation z

[actuator_length,~] = arclength(x,y,z,'s')   % length of the robot


%% Figures

figure(2)
subplot(1,2,1)
plot3(x,y,-z)
xlabel('x'); ylabel('y');zlabel('z')
grid on; box on;
subplot(1,2,2)
plot(g(:,5),-g(:,7))
grid on;box on
xlabel('x');ylabel('z')

toc

