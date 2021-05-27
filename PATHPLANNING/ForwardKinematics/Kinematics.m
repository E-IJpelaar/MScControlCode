clear all;close all;clc;tic
%% Parameters
shape = "legendre";   % poly = polynomial, cheby = chebyshev, legendre = legendre
Nmode = 1;            % # shape functions to approximate strain/curvature

L0 = 70e-3;         % undeformed length of actuator

q = [6,0.2];         % q(t) = q(0) kappa;epsilon

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
Q0 = rotm2quat(eye(3));
r0 = zeros(3,1);
g0 = [Q0(:);r0];

[l, g] = ode45(@(l,g) forwardKinematics(l,g,q,Ba,shape,Nmode,L0),[0 L0],g0); % solve forward kinematics

%% Interpret data
R = g(:,1:4);       % robot's rotation expressed in quaternions
x = g(:,5);         % robot's translation x
y = g(:,6);         % robot's translation y
z = g(:,7);         % robot's translation z

[actuator_length,~] = arclength(x,y,z,'s')   % length of the robot

toc
%% Figures

figure(1)
% subplot(1,2,1)
% plot3(x,y,-z)
% xlabel('x'); ylabel('y');zlabel('z')
% grid on; box on;
% subplot(1,2,2)
% hold on
m2mm = 1e3;
plot(x*m2mm,z*m2mm,'r','LineWidth',2)
grid on;box on;hold on;
% plot(x(end)*m2mm,z(end)*m2mm,'rx','LineWidth',2,'MarkerSize',16)
xlabel('x position [mm]','FontSize',12);ylabel('y position [mm]','FontSize',12)
% axis([0 40 0 55]) 
% legend('Kinematic configuration for q = [\kappa,\epsilon]^T = [17,0.1]^T')
axis equal; 
% axis tight;


toc


