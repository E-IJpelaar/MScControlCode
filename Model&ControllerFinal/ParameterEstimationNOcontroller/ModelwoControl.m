clear;close all;clc;tic


% H = [0.0206 , -0.0206;
%      0.1605 ,  0.1606]; % changed mapping
%  
H = [0.0018 , -0.0018;
    0.1453  ,  0.1453]; % old mapping

dt = 25e-3;
t_begin = 0;
t_end = 182;
 

%% Parameters
Nmode = 1;
space_step = 15;
shape = 'cheby';
%% Parameters
% geometric properties
L0 = 0.070;             % [m] initial length
m  = 0.0332;            % [kg] actuator weight
w  = 0.064;             % [m] width of actuator
d  = 0.025;             % [m] depth of the actuator

% Damping matrix             
D_k = 4e-5;               % [Nsm]Linear damping on bending (order E-5)
% D_k = 6.75e-6;          % [Nsm]Linear damping on bending (order E-5)
% D_e = 0.3;                % [Ns/m]Linear damping on elongation (order E-3)
% D_e = 3;
D_e = 2.5e-2;
D   = diag([D_k,D_e]);    % Damping matrix

%% Load input signal, pressure signal

data = load('16069V.txt');
Voltstep = 9;

m2mm = 1e3;

texp = data(:,1);
p1 = data(:,2);
p01 = data(:,3);
p2 = data(:,4);
p02 = data(:,5);
u1 = data(:,6);
u2 = data(:,7);
x = data(:,8)*m2mm;
y = data(:,9)*m2mm;
kappa = data(:,10);
epsilon = data(:,11);
angle = data(:,12);

% figure(11)
% plot(texp,epsilon)
% hold on;grid on;
% 
% figure(12)
% plot(texp,angle)
% hold on;grid on;
% 
% figure(13)
% plot(texp,kappa)
% hold on;grid on;


%% Initial conditions 
% initial conditions 
rot0 = 0;                   % [deg]  initial rotation
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
T = t_begin:dt:t_end;
x0 = [q0 dq0 p0];                       % initial condition vector
[t,x] = ode23(@(t,x) nonLinearDynamicModelPressureModelParameterEstimation(t,x,D,H,L0,m,w,d,Nmode,shape,space_step,dt,Voltstep),T,x0);

%% Data extraction

k = x(:,1);  % elongation
e = x(:,2);  % curvature
for ii = 1:length(k)   
r_pos = ts2cs(k(ii),e(ii),L0);
x_pos(ii) = r_pos(1);
y_pos(ii) = r_pos(2);
end


L = L0*(1+e);    % actuator length
rot = rad2deg(k.*(L0.*(1+e))); % rotation

dk = x(:,3); % curvature rate
de = x(:,4); % elongation rate
p1sim = x(:,5);
p2sim = x(:,6);

figure(2)
plot(t,e)
hold on;grid on;
plot(texp,epsilon)
legend('Simulation','Experiment')
xlabel('Time [s]');ylabel('Elongation')


figure(3)
plot(t,rot)
hold on;grid on;
plot(texp,angle-angle(1))
% plot(texp,angle)
legend('Simulation','Experiment')
xlabel('Time [s]');ylabel('Rotation [deg]')


figure(4)
plot(t,p1sim)
hold on;grid on;
plot(texp,(p1-p01)/10)
plot(t,p2sim)
plot(texp,(p2-p02)/10)
xlabel('Time [s]');ylabel('Pressure  [kPa]')
legend('Simulation p1','Experiment p1','Simulation p2','Experiment p2')


% figure(5)
% plot(t(4002:end),rot(4002:end))
% hold on;grid on;
% plot(texp(1001:end),angle( 1001:end))


toc




