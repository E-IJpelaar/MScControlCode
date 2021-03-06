clear all;close all;clc;
%% Paramters
L_act = 64.5e-3;
A_eff = 0.1462;   % [m^2] effective area on which pressure acts (FEM determined)
r = 12.56e-3;     % [m]   lever on which force acts (geometrically determined)

H = [A_eff  ,  A_eff   ; 
     A_eff*r, -A_eff*r];  % Mapping matrix p [kPa] => F/M [N/Nm]

D_e = 9;                  % Linear damping on elongation
D_k = 5.93e-2;            % Linear damping on bending
D = diag([D_e,D_k]);      % Damping matrix
invD = inv(D);            % Inverse of Damping matrix
%% Rewrite system dq = D^-1(Hp - Kq)

x0 = [0,17];
[t,x] = ode45(@(t,x) nonlinmodel(t,x,invD,H),[0 2],x0);


figure(1)
plot(t,x(:,1),'LineWidth',1.5)
hold on;grid on;
plot(t,x(:,2),'LineWidth',1.5)
xlabel('Time [s]');ylabel('Output')
legend('\epsilon','\kappa')


figure(2)
plot(t,x(:,1)*L_act,'LineWidth',1.5)
hold on;grid on;
plot(t,x(:,2).*x(:,1).*L_act,'LineWidth',1.5)
xlabel('Time [s]');ylabel('Output')
legend('Elongation [mm]','Rotation [deg]')









