function [x,z] = funcKinematicsXZ(q,L0,Nmode,shape)
%% Contrained strain/curvature, 0 = contrained 1 = free
K1 = 0;  % curvatures
K2 = 1;
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
Bc = zeros(length(xi_ac),length(xi_ac)-n);  % pre-alociation

a = find(xi_ac == 1);                       % inidices active DOF
c = find(xi_ac == 0);                       % inidices contrained DOF
for ii = 1:length(a)
    Ba(a(ii),ii) =1;                        % active DOF
end
for ii = 1:length(c)
    Bc(c(ii),ii) =1;                        % complementary matrix
end

if Nmode*n ~= length(q)                     % throw error when q is not of satisfactory length
    error('q should have length Nmode*active strains')
end

%% Forward kinematics
Q0 = rot2quat(eye(3));
r0 = zeros(3,1);
g0 = [Q0;r0];

[l, g] = ode45(@(l,g) forwardKinematics(l,g,q,Ba,shape,Nmode,L0),[0 L0],g0); % solve forward kinematics
%% Interpret data
R = g(:,1:4);       % robot's rotation expressed in quaternions
x = g(:,5);         % robot's translation x
y = g(:,6);         % robot's translation y
z = g(:,7);         % robot's translation z