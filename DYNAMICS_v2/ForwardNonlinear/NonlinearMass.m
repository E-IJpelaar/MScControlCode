clear all;close all;clc;tic
mm2m = 1e-3; g2kg =1e-3;
load mapping.mat; clear datap;
H = flip(H);
%% 
Nmode = 1;
space_step = 20;
shape = 'cheby';
%% Parameters
% geometric properties
L0 = 64.5*mm2m;           % [m] initial length
m  = 30*g2kg;            % [kg] actuator weight
w = 64*mm2m;              % [m] width of actuator
d = 25*mm2m;              % [m] depth of the actuator

% Damping matrix
% D_k = 4e-5;
% D_e = 0.3;               
D_k = 2e-6;           
D_e = 0.009;
D = diag([D_k,D_e]);      % Damping matrix
invD = inv(D);            % Inverse of damping matrix
 
%% Initial conditions 
% initial conditions 
e0   = 0.5;                        % [-]    initial elongation
rot0 = 45;                         % [deg]  initial rotation
k0   = deg2rad(rot0)/(L0*(1+e0));  % [1/m] initial curvature
q0   = [k0 e0];

de0   = 0;                           % [1/s]   initial elongation rate 
drot0 = 0;                           % [deg/s] initial rotation rate
dk0   = (deg2rad(drot0) - (k0*L0*de0))/L0;  %[1/ms] initial curvature rate
dq0   = [dk0 de0];

% solve SS-model
x0 = [q0 dq0];                       % initial condition vector
[t,x] = ode23t(@(t,x) nonLinearDynamicModel(t,x,D,H,L0,m,w,d,Nmode,shape,space_step),[0:0.0001:0.2],x0);
toc
%% Data extraction
k = x(:,1);  % curvature
e = x(:,2);  % elongation


L = L0*(1+e);    % actuator length
rot = rad2deg(k.*(L0.*(1+e))); % rotation

dk = x(:,3); % curvature rate
de = x(:,4); % elongation rate

%% Figure
figure(1)
yyaxis left
plot(t,e,'LineWidth',1.5)
ylabel('Elongation \epsilon [-]','FontSize',12)
hold on;grid on;

yyaxis right
plot(t,k,'LineWidth',1.5)
xlabel('Time [s]','FontSize',12);ylabel('Curvature \kappa [1/m]','FontSize',12)
legend('\epsilon [-]','\kappa [1/m]','FontSize',12)

 
figure(2)
yyaxis left
plot(t,L,'LineWidth',1.5)
hold on;grid on;
ylabel('Actuator length [m]')
ylim([0 0.12])

yyaxis right
plot(t,rot,'LineWidth',1.5)
xlabel('Time [s]');ylabel('Rotation [deg]')


figure(3)
yyaxis left
plot(t,de,'LineWidth',1.5)
ylabel('Elongation \epsilon [-]','FontSize',12)
hold on;grid on;

yyaxis right
plot(t,dk,'LineWidth',1.5)
xlabel('Time [s]','FontSize',12);ylabel('Curvature \kappa [1/m]','FontSize',12)
legend('\epsilon [-]','\kappa [1/m]','FontSize',12)


