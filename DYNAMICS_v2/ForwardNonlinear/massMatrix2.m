function [Mq] = massMatrix2(e,k,L0,m,w,d,Nmode,shape,space_step,J_cor)
q = [e,k];       % modal coordinates (check order first curvature then elongation)
%% Contrained strain/curvature, 0 = contrained 1 = free
E1 = 1;  % strains
E2 = 0;
E3 = 0;
K1 = 0;  % curvatures
K2 = 1;
K3 = 0;
E = [E1;E2;E3];
K = [K1;K2;K3];
xi_ac = [E;K];   % xi with a(ctive) and c(onctrained) strains/curvatures
%% Create actuation matrix Ba and its complementary Bc
n = length(find(xi_ac == 1));               % # active curvatures/strains
Ba = zeros(length(xi_ac),n);                % pre-alociation
a = find(xi_ac == 1);                       % inidices active DOF
    for ii = 1:length(a)
        Ba(a(ii),ii) = 1;                   % active DOF
    end   
    
%% New implementation
% 
d_sigma = L0/space_step;
sigma = 0;

Q0 = rot2quat(eye(3)); % initial no rotation 
r0 = zeros(3,1);       % position vector is 0
g0 = [Q0;r0];          % initial condition on g
g =g0;

J0 = zeros(length(xi_ac),length(q));       % start of with empty jacobian matrix
Jt = J0;

M0 = zeros(length(q),length(q));
M = M0;

% Gq = zeros(n,1);
 
while sigma <= L0  % integral on domain 0 to L0, shape function will max be equal to 1
    
    [dg1,dJt1,dM1] = massODE2(sigma,g,Jt,q,Ba,shape,Nmode,L0,m,w,d,d_sigma,J_cor);
    [dg2,dJt2,dM2] = massODE2(sigma+(2/3*d_sigma),g+(2/3)*d_sigma*g,Jt+(2/3)*d_sigma*Jt,q,Ba,shape,Nmode,L0,m,w,d,d_sigma,J_cor);
    
    sigma = sigma+d_sigma;
    g = g + (d_sigma/4 * (dg1 + 3*dg2));
    Jt = Jt + (d_sigma/4 * (dJt1 + 3*dJt2));
    M = M + (d_sigma/4 * (dM1 + 3*dM2));
%     Gq = Gq + (d_sigma/4) * (dGq1 + 3*dGq2);
    
end
Mq = M
