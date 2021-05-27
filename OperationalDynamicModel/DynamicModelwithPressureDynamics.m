clear;close all;clc;tic

H = [0.0018 , -0.0018;
     0.1453 ,  0.1453];
%  
 
% H = [0.0206 , -0.0206;
%      0.1808 ,  0.1808];
 
dt = 1e-3;
t_end = 20;
 
global tau_error_sum  p_error_sum U1 U2 time pset1 pset2 tau1 tau2;
tau_error_sum = zeros(2,1);
p_error_sum = zeros(2,1);
U1 = zeros(1,length(0:dt:t_end)+1);
U2 = zeros(1,length(0:dt:t_end)+1);

pset1 = zeros(1,length(0:dt:t_end)+1);
pset2 = zeros(1,length(0:dt:t_end)+1);

tau1 = zeros(1,length(0:dt:t_end)+1);
tau2 = zeros(1,length(0:dt:t_end)+1);

time = zeros(1,length(0:dt:t_end)+1);
%% Parameters
Nmode = 1;
space_step = 15;
shape = 'cheby';
%% Parameters
% geometric properties
L0 = 0.070;            % [m] initial length
m  = 0.0332;            % [kg] actuator weight
w  = 0.054;             % [m] width of actuator
d  = 0.025;             % [m] depth of the actuator

% Damping matrix             
D_k = 4e-5;               % [Nsm]Linear damping on bending (order E-5)
D_e = 0.3;                % [Ns/m]Linear damping on elongation (order E-3)
D   = diag([D_k,D_e]);    % Damping matrix

%% Reference
% set refernce
r_ref3 = ts2cs3(4,0.2,L0);
r_ref = [r_ref3(2);r_ref3(3)];

% Kp = diag([5000,5000]); % current best tuning
% Ki = diag([1500,3500]);

Kp = diag([1500,1500]); % current best tuning
Ki = diag([1500,1500]);

% effects lower I, higher I, Kp write down for report.


% Kpp = diag([1,1]);
% Kip = diag([0,0]);

Kpp = diag([1,1]);
Kip = diag([0.15,0.15]);



%% Initial conditions 
% initial conditions 
rot0 = 0;                         % [deg]  initial rotation
e0   = 0;                          % [-]    initial elongation
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
T = 0:dt:t_end;
x0 = [q0 dq0 p0];                       % initial condition vector
[t,x] = ode23(@(t,x) nonLinearDynamicModelPressureModel(t,x,D,H,L0,m,w,d,Nmode,shape,space_step,r_ref,dt,Kp,Ki,Kpp,Kip),T,x0);

%% Data extraction

k = x(:,1);  % elongation
e = x(:,2);  % curvature
for ii = 1:length(k)   
r_pos = ts2cs(k(ii),e(ii),L0);
x_pos(ii) = r_pos(1);
y_pos(ii) = r_pos(2);
end


figure(100)
plot(t,x_pos,'LineWidth',1.5)
hold on;grid on;
plot(t,r_ref(1)*ones(1,length(x_pos)),'r-','LineWidth', 1.5)
xlabel('Time [s]');ylabel('Position [m]')
legend('x - position','Reference','FontSize',12)


% figure(11)
plot(t,y_pos,'LineWidth',1.5)
hold on;grid on;
plot(t,r_ref(2)*ones(1,length(y_pos)),'r-','LineWidth', 1.5)
xlabel('Time [s]');ylabel('Position [m]')
legend('y - position','Reference','FontSize',12)



L = L0*(1+e);    % actuator length
rot = rad2deg(k.*(L0.*(1+e))); % rotation

dk = x(:,3); % curvature rate
de = x(:,4); % elongation rate


%% Modal coordinates
% figure(1)
% yyaxis left
% plot(t,e,'LineWidth',1.5)
% ylabel('\epsilon [-]')
% hold on;grid on;
% plot(t,(r_ref(2)/L0)-1*ones(length(e),1))
% 
% yyaxis right
% plot(t,k,'LineWidth',1.5)
% xlabel('Time [s]');ylabel('\kappa [1/m]')
% legend('\epsilon [-]','\kappa [1/m]')

%% Modal coordinate velocity

% figure(2)
% yyaxis left
% plot(t,de,'LineWidth',1.5)
% ylabel('$\dot{\epsilon}$ [1/s]','Interpreter','Latex')
% hold on;grid on;
% 
% yyaxis right
% plot(t,dk,'LineWidth',1.5)
% xlabel('Time [s]');ylabel('$\dot{\kappa}$ [1/ms]','Interpreter','Latex')
% legend('$\dot{\epsilon}$ [1/s]','$\dot{\kappa}$ [1/ms]','Interpreter','Latex')
% % 

%% Actuator length and rotation
% figure(3)
% yyaxis left
% plot(t,L,'LineWidth',1.5)
% hold on;grid on;
% ylabel('Actuator length [m]')
% ylim([0 0.12])
% plot(t,(r_ref(2))*ones(length(L),1))
% 
% 
% yyaxis right
% plot(t,rot,'LineWidth',1.5)
% hold on;grid on;
% % plot(t,r_ref(1)*ones(length(rot),1))
% xlabel('Time [s]');ylabel('Rotation [deg]')
% % legend('With Coriolis & Gravity','Without Coriolis & Gravity')


%% Error
% figure(14)
% plot(t,r_ref(1)-x_pos,'r','LineWidth',1.5)
% hold on; grid on;
% plot(t,r_ref(2)-y_pos,'b','LineWidth',1.5)
% xlabel('Time [s]');ylabel('Position error [m]')

idx = find(U1 ~= 0);
timeU1 = t(idx);
U1p = U1(idx);

idx = find(U2 ~= 0);
timeU2 = t(idx);
U2p = U2(idx);

idx = find(pset1 ~= 0);
timepset1 = t(idx);
pset1p = pset1(idx);

idx = find(pset2 ~= 0);
timepset2 = t(idx);
pset2p = pset2(idx);

idx = find(tau1 ~= 0);
timetau1 = t(idx);
tau1p = tau1(idx);

idx = find(tau2 ~= 0);
timetau2 = t(idx);
tau2p = tau2(idx);


figure(1)
subplot(3,1,1)
plot(t,r_ref(1)-x_pos,'b','LineWidth',1.5)
hold on;grid on;
yline(0,'r--')
ylabel('Error x position [m]')

subplot(3,1,2)
plot(t,r_ref(2)-y_pos,'b','LineWidth',1.5)
hold on;grid on;
yline(0,'r--')
ylabel('Error y position [m]')

subplot(3,1,3)
plot(timeU1,U1p,'r','LineWidth',1.5)
hold on; grid on;
plot(timeU2,U2p,'b','LineWidth',1.5)
xlabel('Time [s]');ylabel('Control Input [V]')
legend('V_1','V_2')


figure(2)
plot(timepset1,pset1p,'r')
hold on; grid on;
plot(timepset2,pset2p,'b')
plot(t,x(:,5),'r','LineWidth',1.5)
plot(t,x(:,6),'b','LineWidth',1.5)
xlabel('Time [s]');ylabel('Pressure [kPa]')
legend('p_{ref1}','p_{ref2}','p_1','p_2')

figure(3)
yyaxis left
plot(timetau1,tau1p,'b','LineWidth',1.5)
hold on; grid on;
ylabel('Input Moment')

yyaxis right
plot(timetau2,tau2p,'r','LineWidth',1.5)
xlabel('Time [s]');ylabel('Input Force')


toc