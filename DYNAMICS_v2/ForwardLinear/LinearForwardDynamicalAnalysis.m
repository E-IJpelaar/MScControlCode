clear all;close all;clc;
mm2m = 1e-3;
%% Parameters
% actuator length 
L0 = 64.5*mm2m;           % [m]

% Constant mass matrix 
Me  = 0.0177;             % [kg] Mass
Jk  = 1.21e-5;            % [kgm^2] Inertia
M = diag([Me,Jk]);        % Mass matrix
invM = inv(M);            % Inverse mass matrix

% Damping matrix
D_e = 0.2;                 % Linear damping on elongation
D_k = 0.02;                 % Linear damping on bending
D = diag([D_e,D_k]);      % Damping matrix
invD = inv(D);            % Inverse of damping matrix

% Stiffness matrix
Ke = 35;                  % Elongation stiffness (+/-)
Kk = 0.12;                % Rotation stiffness (+/-)
K = diag([Ke,Kk]);        % Stiffness matrix


% Pressure mapping
A_eff = 0.1462;           % [m^2] effective area on which pressure acts (FEM determined)
r = 12.56e-3;             % [m]   lever on which force acts (geometrically determined)

H = [A_eff  ,  A_eff   ; 
     A_eff*r, -A_eff*r];  % Mapping matrix p [kPa] => F/M [N/Nm]

%% State-space formulation
O2 = zeros(2,2);
I2 = eye(2,2);

A = [ O2     , I2     ;
     -invM*K ,-invM*D];
 
B = [ O2;
      invM*H   ];
  
%% Solve SS
% initial conditions 

e0   = 0;                          % [-]    initial elongation
rot0 = 20;                          % [deg]  initial rotation
k0   = deg2rad(rot0)/(L0*(1+e0));  % [1/m] initial curvature
q0   = [e0 k0];

de0   = 0;                           % [1/s]   initial elongation rate 
drot0 = 0;                           % [deg/s] initial rotation rate
dk0   = (deg2rad(drot0) - (k0*L0*de0))/L0;  %[1/ms] initial curvature rate
dq0   = [de0 dk0];

x0 = [q0 dq0];                       % initial condition vector
[t,x] = ode23t(@(t,x) forwardDynamics(t,x,A,B),[0 0.5],x0);
  
%% Data extraction

e = x(:,1);  % elongation
k = x(:,2);  % curvature

L = L0*(1+e);    % actuator length
rot = rad2deg(k.*(L0.*(1+e))); % rotation

de = x(:,3); % elongation rate
dk = x(:,4); % curvature rate

%% Figure
figure(1)
plot(t,e,'LineWidth',1.5)
hold on;grid on;
plot(t,k,'LineWidth',1.5)
xlabel('Time [s]');ylabel('Output')
legend('\epsilon [-]','\kappa [1/m]')

 
figure(2)
yyaxis left
plot(t,L,'LineWidth',1.5)
hold on;grid on;
ylabel('Actuator length [m]')
ylim([0 0.2])

yyaxis right
plot(t,rot,'LineWidth',1.5)
xlabel('Time [s]');ylabel('Rotation [deg]')








