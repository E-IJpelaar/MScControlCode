clear all;close all;clc;tic
load mapping.mat;
H = flip(H);
%% Parameters
Nmode = 1;
space_step = 20;
shape = 'cheby';
corgrav = 0; % include gravity and coriolis effects (1)
%% Parameters
% geometric properties
L0 = 0.0645;            % [m] initial length
m  = 0.0332;            % [kg] actuator weight
w  = 0.054;             % [m] width of actuator
d  = 0.025;             % [m] depth of the actuator

% Damping matrix             
D_k = 1e-6;               % [Nsm]Linear damping on bending (order E-5)
D_e = 0.8;                % [Ns/m]Linear damping on elongation (order E-3)
D = diag([D_k,D_e]);      % Damping matrix

%% Initial conditions 
% initial conditions 
rot0 = 40;                          % [deg]  initial rotation
e0   = 0.1;                          % [-]    initial elongation
k0   = deg2rad(rot0)/(L0*(1+e0));  % [1/m] initial curvature
q0   = [k0 e0];

drot0 = 0;                           % [deg/s] initial rotation rate
de0   = 0;                           % [1/s]   initial elongation rate 
dk0   = (deg2rad(drot0) - (k0*L0*de0))/L0;  %[1/ms] initial curvature rate
dq0   = [dk0 de0];

% solve SS-model
x0 = [q0 dq0];                       % initial condition vector
[t,x] = ode23t(@(t,x) nonLinearDynamicModelV2(t,x,D,H,L0,m,w,d,Nmode,shape,space_step,corgrav),[0 0.5],x0);
toc
%% Data extraction

k = x(:,1);  % elongation
e = x(:,2);  % curvature

L = L0*(1+e);    % actuator length
rot = rad2deg(k.*(L0.*(1+e))); % rotation

dk = x(:,3); % curvature rate
de = x(:,4); % elongation rate


%% Figure
figure(1)
yyaxis left
plot(t,e,'LineWidth',1.5)
ylabel('\epsilon [-]')
hold on;grid on;

yyaxis right
plot(t,k,'LineWidth',1.5)
xlabel('Time [s]');ylabel('\kappa [1/m]')
legend('\epsilon [-]','\kappa [1/m]')

figure(2)
yyaxis left
plot(t,de,'LineWidth',1.5)
ylabel('$\dot{\epsilon}$ [1/s]','Interpreter','Latex')
hold on;grid on;

yyaxis right
plot(t,dk,'LineWidth',1.5)
xlabel('Time [s]');ylabel('$\dot{\kappa}$ [1/ms]','Interpreter','Latex')
legend('$\dot{\epsilon}$ [1/s]','$\dot{\kappa}$ [1/ms]','Interpreter','Latex')

 
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


