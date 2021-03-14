clear all;close all;clc;

g2kg = 1e-3;
mm2m = 1e-3;
%% Parameters

m = 120*g2kg;           % [kg] mass of actuator 
L0 = 64.5*mm2m;          % [m] length of actuator
e = 0;
w = 64*mm2m;
d = 25*mm2m;
h = 1*mm2m; %height of a slice sigma
[m_sigma,Jxx,Jyy,Jzz] = inertiaRectangle(m,L0,e,w,d,h);


%% IMPORTANT NOTE 
% K2 = 0 and K1 = 1. Previous scripts had K1 = 0 and K2 = 1
% As we rotate around x-axis instead of y-axis. Previous results are still
% valid but work in the x-z plane, from now on we work in the y-z plane.
%% Contrained strain/curvature, 0 = contrained 1 = free
K1 = 1;  % curvatures
K2 = 0;
K3 = 0;
E1 = 0;  % strains
E2 = 0;
E3 = 1;
K = [K1;K2;K3];
E = [E1;E2;E3];
xi_ac = [K;E];   % xi with a(ctive) and c(onctrained) strains/curvatures
%% Create actuation matrix Ba and its complementary Bc
n = length(find(xi_ac == 1));               % # active curvatures/strains
Ba = zeros(length(xi_ac),n);                % pre-alociation
a = find(xi_ac == 1);                       % inidices active DOF
for ii = 1:length(a)
    Ba(a(ii),ii) =1;                        % active DOF
end


M = diag([m_sigma,m_sigma,m_sigma,Jxx,Jyy,Jzz]);


l = 0:0.01:L0;

for ii =1:length(l)

                Baphi_s = shapeValue(shape,Nmode,l(ii),Ba);    % Determine Ba*Phi_s for each sigma
                Adg = adjointG(R(ii,:),r(ii,:));               % Calculate Adg for each sigma
                J = J + Adg*Baphi_s;                           % Add contribution of each delta sigma to total Jacobian



Mq = J.'*M*J;