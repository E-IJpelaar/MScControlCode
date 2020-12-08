clear all;close all;clc;
%% Parameters
% geometric properties
L0 = 64.5e-3;             % [m] initial length
A_eff = 0.1462;           % [m^2] effective area on which pressure acts (FEM determined)
r = 12.56e-3;             % [m]   lever on which force acts (geometrically determined)

% Constant mass matrix 
Me  = 0.0177;             % [kg] Mass
Jk  = 1.21e-5;            % [kgm^2] Inertia
M = diag([Me,Jk]);        % Mass matrix
invM = inv(M);            % Inverse mass matrix

% Damping matrix
D_e = 0.1;                 % Linear damping on elongation
D_k = 0.01;                 % Linear damping on bending
D = diag([D_e,D_k]);      % Damping matrix
invD = inv(D);            % Inverse of damping matrix

% Pressure mapping
H = [A_eff  ,  A_eff   ; 
     A_eff*r, -A_eff*r];  % Mapping matrix p [kPa] => F/M [N/Nm]
 
%% Initial conditions 
% position level
e0   = 0;                            % [-]    initial elongation
% rot0 = 20;                              % [deg]  initial rotation
k0   = 0.6;%deg2rad(rot0)/(L0*(1+e0));     % [1/m] initial curvature
q0   = [e0 k0];

% velocity level
de0   = 0;                           % [1/s]   initial elongation rate 
dk0   = k0*L0*de0;                   % [1/ms] initial curvature rate
dq0   = [de0 dk0];

% solve SS-model
x0 = [q0 dq0];                       % initial condition vector
[t,x] = ode23t(@(t,x) NonLinearDynamics(t,x,invM,D,H),[0 2.5],x0);
 
%% Figures
figure(3)
plot(t,x(:,1),'LineWidth',1.5)
hold on;grid on;
plot(t,x(:,2),'LineWidth',1.5)
xlabel('Time [s]','FontSize',12);ylabel('Output','FontSize',12)
legend('\epsilon','\kappa','FontSize',12)
% axis equal

figure(2)
plot(t,((x(:,1)+1)*L0)*1e3,'LineWidth',1.5)
hold on;grid on;
plot(t,rad2deg((L0.*(x(:,1)+1).*x(:,2))./(k0*L0)),'LineWidth',1.5)
xlabel('Time [s]');ylabel('Output')
legend('Actuator length [mm]','Rotation [deg]')

 






