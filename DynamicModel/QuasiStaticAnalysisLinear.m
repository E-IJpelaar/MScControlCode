clear all;close all;clc;
%% Paramters
A_eff = 0.1462;   % [m^2] effective area on which pressure acts (FEM determined)
r = 12.56e-3;     % [m]   lever on which force acts (geometrically determined)

H = [A_eff  ,  A_eff   ; 
     A_eff*r, -A_eff*r];  % Mapping matrix p [kPa] => F/M [N/Nm]

 
K_e = 50;                 % Linear elongation stiffness [N]
K_k = 20;                 % Linear bending stiffness    [Nm^2]
K = diag([K_e,K_k]);      % Stiffness matrix

D_e = 10;                  % Linear damping on elongation
D_k = 5;                  % Linear damping on bending
D = diag([D_e,D_k]);      % Damping matrix

%% Rewrite system dq = D^-1(Hp - Kq)
A = -D\K;
B = D\H;

x0 = [0.56,0];
[t,x] = ode45(@(t,x) linmodel(t,x,A,B),[0 2],x0);


figure(1)
plot(t,x(:,1))
hold on;grid on;
plot(t,x(:,2))
xlabel('Time [s]');ylabel('Output')
legend('\epsilon','\kappa')







