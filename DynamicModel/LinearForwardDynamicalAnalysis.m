clear all;close all;clc;
%% Parameters
% actuator length
L0 = 64.5;                % [mm]

% Mass matrix
Me  = 0.0177;             % [kg] Mass
Jk  = 1.21e-5;            % [kgm^2] Inertia
Jk  = 0.0121;
M = diag([Me,Jk]);        % Mass matrix
invM = inv(M);            % Inverse mass matrix

% Damping matrix
D_e = 60;                 % Linear damping on elongation
D_k = 20;                 % Linear damping on bending
D = diag([D_e,D_k]);      % Damping matrix
invD = inv(D);            % Inverse of damping matrix

% Stiffness matrix
Ke = 20;                  % Elongation stiffness
Kk = 50;                  % Rotation stiffness 
K = diag([Ke,Ke]);        % Stiffness matrix


% Pressure mapping
A_eff = 0.1462;           % [m^2] effective area on which pressure acts (FEM determined)
r = 12.56e-3;             % [m]   lever on which force acts (geometrically determined)

H = [A_eff  ,  A_eff   ; 
     A_eff*r, -A_eff*r];  % Mapping matrix p [kPa] => F/M [N/Nm]

%% State-space formulation
O2 = zeros(2,2);
I2 = eye(2,2);

A = [ O2     , I2     ;
     -invM\K ,-invM\D];
 
B = [ O2;
      invM\H   ];
  
%% Solve SS
% initial conditions 

e0   = 0.1;                          % [-]    initial elongation
rot0 = 1;                           % [deg]  initial rotation
k0   = deg2rad(rot0)/(e0*L0);     % [1/mm] initial curvature
q0   = [e0 k0];

de0   = 0.0;                           % [1/s]   initial elongation rate 
dk0   = k0*L0*de0;                % [1/mms] initial curvature rate
dq0   = [de0 dk0];

x0 = [q0 dq0];                       % initial condition vector
[t,x] = ode45(@(t,x) ForwardDynamics(t,x,A,B),[0 20],x0);
  
 
%% Figures

figure(1)
plot(t,x(:,1),'LineWidth',1.5)
hold on;grid on;
plot(t,x(:,2),'LineWidth',1.5)
xlabel('Time [s]');ylabel('Output')
legend('\epsilon','\kappa')
% 
% 
figure(2)
plot(t,x(:,1)*L0,'LineWidth',1.5)
hold on;grid on;
plot(t,rad2deg(x(:,2).*x(:,1).*L0),'LineWidth',1.5)
xlabel('Time [s]');ylabel('Output')
legend('Elongation [mm]','Rotation [deg]')
% 
% 
% 






