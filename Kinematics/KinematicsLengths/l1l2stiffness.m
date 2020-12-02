clear all;close all;clc;
%% Parameters
shape = "cheby";   % poly = polynomial, cheby = chebyshev, legendre = legendre
Nmode = 1;            % # shape functions to approximate strain/curvature
L_act = 64.5;
r     = 12.56;
nr1   = r/L_act;
L = 1;                % undeformed length of actuator
q = [0.6;0.6];    % q(t) = q(0)

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
a = find(xi_ac == 1);                       % inidices active DOF
for ii = 1:length(a)
    Ba(a(ii),ii) =1;                        % active DOF
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

%% Extra polate to l1 and l2
l1x = zeros(1,length(x));l1z = zeros(1,length(x));l2x = zeros(1,length(x));l2z = zeros(1,length(x));

for ii = 1:length(l) 
rot_ii = quat2rot(g(ii,1:4));
theta_ii = atan2(rot_ii(3,1),rot_ii(1,1));
l1x(ii) = x(ii) - nr1*cos(theta_ii);
l1z(ii) = z(ii) - nr1*sin(theta_ii);
l2x(ii) = x(ii) + nr1*cos(theta_ii);
l2z(ii) = z(ii) + nr1*sin(theta_ii);
end

%% Calculate lengths
[actuator_length,~] = arclength(x,z,'s');   % length of the robot
[l1,~] = arclength(l1x,l1z,'s');   % length of the robot
[l2,~] = arclength(l2x,l2z,'s');   % length of the robot


figure(2)
plot(x,z,'LineWidth',2)
hold on; grid on;box on;
plot(l1x,l1z,'LineWidth',2)
plot(l2x,l2z,'LineWidth',2)
xlabel('y [-]');ylabel('z [-]')
legend('backbone centre','l_1','l_2')
axis equal


figure(3)
plot(x*L_act,z*L_act,'LineWidth',2)
hold on; grid on;box on;
plot(l1x*L_act,l1z*L_act,'LineWidth',2)
plot(l2x*L_act,l2z*L_act,'LineWidth',2)
xlabel('y [mm]');ylabel('z [mm]')
legend('backbone centre','l_1','l_2')
axis equal

% gamma = [2300.82232787747,2300.80163158766,0.00742041601377047];
% K_kappa = @(q)(gamma(1)+ gamma(2)*((tanh(gamma(3)*q(1)))^2-1));
K_e1 = @(q)(1.1579e+3 + 1.1423e3*(((tanh(3.134e-1*q(2)))^2)-1));
K_e2 = @(q)(1.1579e+3 + 1.1423e3*(((tanh(3.134e-1*q(2)))^2)-1) +  abs(q(1))*(1.1579e+3 + 1.1423e3*(((tanh(3.134e-1*q(2)))^2)-1)));
% q = [1,0];
% F = K_e(q)*q(2);
% M = K_kappa(q)*q(1)

q = zeros(2,length(linspace(-1,1,1000)));
q(1,:) = linspace(10,-10,1000);%zeros(1,1000);
q(2,:) = linspace(-10,10,1000);

for ii = 1:length(q)
    K1(ii) = K_e1([q(1,ii),q(2,ii)]);
    K2(ii) = K_e2([q(1,ii),q(2,ii)]);

    
end

figure(5)
plot(q(2,:),K1)
grid on;box on; hold on;
plot(q(2,:),K2)
legend('normal','adapted')

figure(6)
plot(q(1,:),K1)
grid on;box on; hold on;
plot(q(1,:),K2)
legend('normal','adapted')

% open('elongstiffness.fig')
% open('rotstiffness.fig')

