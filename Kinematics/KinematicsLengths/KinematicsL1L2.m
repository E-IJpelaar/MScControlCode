clear all;close all;clc;
%% Parameters
shape = "cheby";   % poly = polynomial, cheby = chebyshev, legendre = legendre
Nmode = 3;            % # shape functions to approximate strain/curvature
L_act = 64.5;
r     = 12.56;
nr1   = r/L_act;
L = 1;                % undeformed length of actuator
q = [0;-0.2;1;0;0.2;-0.4];    % q(t) = q(0)

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
Q0 = rot2quat(eye(3));
r0 = zeros(3,1);
g0 = [Q0;r0];

[l, g] = ode45(@(l,g) forwardKinematics(l,g,q,Ba,shape,Nmode),[0 L],g0); % solve forward kinematics

%% Interpret data
R = g(:,1:4);       % robot's rotation expressed in quaternions
x = g(:,5);         % robot's translation x
y = g(:,6);         % robot's translation y
z = g(:,7);         % robot's translation z

[actuator_length,~] = arclength(x,y,z,'s')   % length of the robot


%% Figures

figure(1)
subplot(1,2,1)
plot3(x,y,-z)
xlabel('x'); ylabel('y');zlabel('z')
grid on; box on;
subplot(1,2,2)
hold on
plot(x,z,'LineWidth',2)
grid on;box on
xlabel('y [-]','FontSize',14);ylabel('z [-]','FontSize',14)
% axis([0 0.15 0 1.2])





%% Extra polate to l1 and l2
l1x = zeros(1,length(x));
l1z = zeros(1,length(x));
l2x = zeros(1,length(x));
l2z = zeros(1,length(x));

for ii = 1:length(l)
rot_ii = quat2rot(g(ii,1:4));
theta_ii = atan2(rot_ii(3,1),rot_ii(1,1));

l1x(ii) = x(ii) - nr1*cos(theta_ii);
l1z(ii) = z(ii) - nr1*sin(theta_ii);

l2x(ii) = x(ii) + nr1*cos(theta_ii);
l2z(ii) = z(ii) + nr1*sin(theta_ii);


end
[l1,~] = arclength(l1x,l1z,'s')   % length of the robot
[l2,~] = arclength(l2x,l2z,'s')   % length of the robot







figure(2)
plot(x,z,'LineWidth',2)
hold on; grid on;box on;
plot(l1x,l1z,'LineWidth',2)
plot(l2x,l2z,'LineWidth',2)
xlabel('x [-]');ylabel('y [-]')
legend('g_0','g_1','g_2')
axis equal


figure(3)
plot(x*L_act,z*L_act,'LineWidth',2)
hold on; grid on;box on;
plot(l1x*L_act,l1z*L_act,'LineWidth',2)
plot(l2x*L_act,l2z*L_act,'LineWidth',2)
xlabel('x [mm]');ylabel('y [mm]')
legend('l_0','l_1','l_2')
axis equal
