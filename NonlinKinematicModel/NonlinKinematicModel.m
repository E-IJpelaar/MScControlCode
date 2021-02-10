clear all;close all;clc;tic;
mm2m = 1e-3; g2kg =1e-3;
load mapping.mat;
H = flip(H); % [rotation;elongation] mapping

%% Parameters
% geometric properties
Nmode = 1;
shape = 'cheby';
L0 = 64.5*mm2m;           % [m] initial length
space_step = 20;

r = [0;L0+0.03;0];


% Damping matrix             
D_k = 1e-4;               % [Nsm]Linear damping on bending (order E-5)
D_e = 0.8;                % [Ns/m]Linear damping on elongation (order E-3)
D = diag([D_k,D_e]);      % Damping matrix
invD = inv(D);
%% Initial conditions 
% initial conditions 
rot0 = 0;                          % [deg]  initial rotation
e0   = 0;                          % [-]    initial elongation
k0   = deg2rad(rot0)/(L0*(1+e0));  % [1/m] initial curvature
q0   = [k0 e0];
% solve SS-model
x0 = q0;                       % initial condition vector
[t,x] = ode23t(@(t,x) KinematicModel(t,x,invD,H,L0,Nmode,shape,space_step,r),[0 2],x0);
toc
%% Data extraction
k = x(:,1);  % elongation
e = x(:,2);  % curvature

L = L0*(1+e);    % actuator length
rot = rad2deg(k.*(L0.*(1+e))); % rotation

%% Figure
% figure(1)
% yyaxis left
% plot(t,e,'LineWidth',1.5)
% ylabel('\epsilon [-]')
% hold on;grid on;
% 
% yyaxis right
% plot(t,k,'LineWidth',1.5)
% xlabel('Time [s]');ylabel('\kappa [1/m]')
% legend('\epsilon [-]','\kappa [1/m]')

 
figure(3)
yyaxis left
plot(t,L,'LineWidth',1.5)
hold on;grid on;
ylabel('Actuator length [m]')
ylim([0 0.12])

yyaxis right
plot(t,rot,'LineWidth',1.5)
xlabel('Time [s]');ylabel('Rotation [deg]')
% legend('With Coriolis & Gravity','Without Coriolis & Gravity')
toc


figure(10)
plot(r(2),r(3),'x','MarkerSize',10)

