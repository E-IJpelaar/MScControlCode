clear all;close all;clc;tic
mm2m = 1e-3; g2kg =1e-3;
load mapping.mat;clear datap % load mapping matrix H
load theta_d.mat;
[p_sim,rot_sim] = steadystateRot(data); % get steadystate rotation results from FEM
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
D_k = 2e-6;               % [Nsm]Linear damping on bending (order E-5)
D_e = 0.8;                % [Ns/m]Linear damping on elongation (order E-3)
D = diag([D_k,D_e]);      % Damping matrix

%% Initial conditions 
% initial conditions 
rot0 = 45;                         % [deg]  initial rotation
e0   = 0.1;                        % [-]    initial elongation
k0   = deg2rad(rot0)/(L0*(1+e0));  % [1/m] initial curvature
q0   = [k0 e0];

drot0 = 0;                           % [deg/s] initial rotation rate
de0   = 0;                           % [1/s]   initial elongation rate 
dk0   = (deg2rad(drot0) - (k0*L0*de0))/L0;  %[1/ms] initial curvature rate
dq0   = [dk0 de0];

% solve SS-model
x0 = [q0 dq0];                       % initial condition vector
[t,x] = ode23t(@(t,x) nonLinearDynamicModelV2(t,x,D,H,L0,m,w,d,Nmode,shape,space_step),[0 0.3],x0);
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
legend('Actuator length','Curvature')
toc

% figure(3)
% plot(t,k,'LineWidth',1)
% hold on;grid on;
% xlabel('Time [s]');ylabel('\kappa [1/m]')
% legend('Non-linear M','Linear M')

