clear all;close all;clc;
mm2m = 1e-3;
load mapping.mat;clear datap % load mapping matrix H
load theta_d.mat;
[p_sim,rot_sim] = steadystateRot(data); % get steadystate rotation results from FEM
%% Parameters
% geometric properties
L0 = 64.5*mm2m;           % [m] initial length

% Constant mass matrix 
Me  = 0.0177;             % [kg] Mass
Jk  = 1.21e-4;            % [kgm^2] Inertia
M = diag([Me,Jk]);        % Mass matrix
invM = inv(M);            % Inverse mass matrix

% Damping matrix
D_e = 0.50;                % Linear damping on elongation
D_k = 0.00015;            % Linear damping on bending
D = diag([D_e,D_k]);      % Damping matrix
invD = inv(D);            % Inverse of damping matrix
 
%% Initial conditions 
% initial conditions 
e0   = 0.2;                          % [-]    initial elongation
rot0 = 0;                         % [deg]  initial rotation
k0   = deg2rad(rot0)/(L0*(1+e0));  % [1/m] initial curvature
q0   = [e0 k0];

de0   = 0;                           % [1/s]   initial elongation rate 
drot0 = 0;                           % [deg/s] initial rotation rate
dk0   = (deg2rad(drot0) - (k0*L0*de0))/L0;  %[1/ms] initial curvature rate
dq0   = [de0 dk0];

% solve SS-model
x0 = [q0 dq0];                       % initial condition vector
[t,x] = ode23t(@(t,x) nonlinearDynamics(t,x,invM,D,H),[0 5],x0);
 
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
ylim([0 0.12])

yyaxis right
plot(t,rot,'LineWidth',1.5)
xlabel('Time [s]');ylabel('Rotation [deg]')
for ii = 1:length(p_sim)
    yline(rot_sim(ii),':',[num2str(p_sim(ii)) ' kPa'])
end

legend('Actuator length','Curvature','Steady-state rotation FEM')
 






