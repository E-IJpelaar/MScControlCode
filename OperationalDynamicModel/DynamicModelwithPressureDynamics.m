clear;close all;clc;tic
load mapping.mat;
H = flip(H);
global tau_error_sum  p_error_sum;
tau_error_sum = zeros(2,1);
p_error_sum = zeros(2,1);
%% Parameters
Nmode = 1;
space_step = 11;
shape = 'cheby';
%% Parameters
% geometric properties
L0 = 0.0645;            % [m] initial length
m  = 0.0332;            % [kg] actuator weight
w  = 0.054;             % [m] width of actuator
d  = 0.025;             % [m] depth of the actuator

% Damping matrix             
D_k = 6e-5;               % [Nsm]Linear damping on bending (order E-5)
D_e = 0.9;                  % [Ns/m]Linear damping on elongation (order E-3)
D   = diag([D_k,D_e]);    % Damping matrix

%% Reference
% set refernce

% r_ref = [0.014;L0+0.0155]; % [x;y]
r_ref3 = ts2cs3(9,0.4,L0)
r_ref = [r_ref3(2);r_ref3(3)];

%% Initial conditions 
% initial conditions 
rot0 = 0;                         % [deg]  initial rotation
e0   = 0.1;                          % [-]    initial elongation
k0   = deg2rad(rot0)/(L0*(1+e0));  % [1/m] initial curvature
q0   = [k0 e0];

drot0 = 0;                           % [deg/s] initial rotation rate
de0   = 0;                           % [1/s]   initial elongation rate 
dk0   = (deg2rad(drot0) - (k0*L0*de0))/L0;  %[1/ms] initial curvature rate
dq0   = [dk0 de0];

% initial conditions pressure
p10 = 0;
p20 = 0;
p0 = [p10 p20];

% solve SS-model
dt = 1e-3;
T = 0:dt:10;
x0 = [q0 dq0 p0];                       % initial condition vector
[t,x] = ode23(@(t,x) nonLinearDynamicModelPressureModel(t,x,D,H,L0,m,w,d,Nmode,shape,space_step,r_ref,dt),T,x0);

%% Data extraction

k = x(:,1);  % elongation
e = x(:,2);  % curvature
for ii = 1:length(k)   
r_pos = ts2cs(k(ii),e(ii),L0);
x_pos(ii) = r_pos(1);
y_pos(ii) = r_pos(2);
end


figure(10)
plot(t,x_pos,'LineWidth',1.5)
hold on;grid on;
plot(t,r_ref(1)*ones(1,length(x_pos)),'r-','LineWidth', 1.5)
xlabel('Time [s]');ylabel('Position [m]')
legend('x - position','Reference','FontSize',12)


figure(11)
plot(t,y_pos,'LineWidth',1.5)
hold on;grid on;
plot(t,r_ref(2)*ones(1,length(y_pos)),'r-','LineWidth', 1.5)
xlabel('Time [s]');ylabel('Position [m]')
legend('y - position','Reference','FontSize',12)



L = L0*(1+e);    % actuator length
rot = rad2deg(k.*(L0.*(1+e))); % rotation

dk = x(:,3); % curvature rate
de = x(:,4); % elongation rate


%% Figure
% figure(1)
% yyaxis left
% plot(t,e,'LineWidth',1.5)
% ylabel('\epsilon [-]')
% hold on;grid on;
% plot(t,(r_ref(2)/L0)-1*ones(length(e),1))


% yyaxis right
% plot(t,k,'LineWidth',1.5)
% xlabel('Time [s]');ylabel('\kappa [1/m]')
% legend('\epsilon [-]','\kappa [1/m]')
% 
figure(2)
yyaxis left
plot(t,de,'LineWidth',1.5)
ylabel('$\dot{\epsilon}$ [1/s]','Interpreter','Latex')
hold on;grid on;

yyaxis right
plot(t,dk,'LineWidth',1.5)
xlabel('Time [s]');ylabel('$\dot{\kappa}$ [1/ms]','Interpreter','Latex')
legend('$\dot{\epsilon}$ [1/s]','$\dot{\kappa}$ [1/ms]','Interpreter','Latex')
% % 
% %  
% figure(3)
% yyaxis left
% plot(t,L,'LineWidth',1.5)
% hold on;grid on;
% ylabel('Actuator length [m]')
% ylim([0 0.12])
% plot(t,(r_ref(2))*ones(length(L),1))
% 
% 
% 
% yyaxis right
% plot(t,rot,'LineWidth',1.5)
% hold on;grid on;
% % plot(t,r_ref(1)*ones(length(rot),1))
% xlabel('Time [s]');ylabel('Rotation [deg]')
% % legend('With Coriolis & Gravity','Without Coriolis & Gravity')



figure(14)
plot(t,r_ref(1)-x_pos,'r','LineWidth',1.5)
hold on; grid on;
plot(t,r_ref(2)-y_pos,'b','LineWidth',1.5)
xlabel('Time [s]');ylabel('Position error [m]')

error = r_ref-[x_pos(end);y_pos(end)]


toc