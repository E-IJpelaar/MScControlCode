function [M,C,G] = massMatrixV2(k,e,dk,de,L0,m,w,d,Nmode,shape,space_step)
q  = [k,e];       % modal coordinates
dq = [dk,de];     % modal coordinates derivatives
%% Contrained strain/curvature, 0 = contrained 1 = free
K1 = 0;  % curvatures
K2 = 1;
K3 = 0;
E1 = 1;  % strains
E2 = 0;
E3 = 0;
K = [K1;K2;K3];
E = [E1;E2;E3];
xi_ac = [K;E];   % xi with a(ctive) and c(onctrained) strains/curvatures
%% Create actuation matrix Ba and its complementary Bc
n = length(find(xi_ac == 1));               % # active curvatures/strains
Ba = zeros(length(xi_ac),n);                % pre-alociation
a = find(xi_ac == 1);                       % inidices active DOF
    for ii = 1:length(a)
        Ba(a(ii),ii) = 1;                   % active DOF
    end   
    
%% MassMatrix,Jacobian,g

Q0 = rot2quat(eye(3)); % initial no rotation 
r0 = zeros(3,1);       % position vector is 0
g0 = [Q0;r0];          % initial condition on g
g =g0;

eta = zeros(6,1);

J0 = zeros(length(xi_ac),length(q));       % start of with empty jacobian matrix
Jt = J0;

M0 = zeros(length(q),length(q));
M = M0;

C0 = zeros(length(q),length(q));
C = C0;

G0 = zeros(length(q),1);
G = G0;

d_sigma = L0/space_step;
L = 0:d_sigma:L0;

for ii = 1:length(L)
    
     sigma = L(ii);
    [deta1,dg1,dJt1,dM1,dC1,dG1] = massODEV2(sigma,g,Jt,eta,q,dq,Ba,shape,Nmode,L0,m,w,d,d_sigma);
    [deta2,dg2,dJt2,dM2,dC2,dG2] = massODEV2(sigma+(2/3)*d_sigma,...
                                             g+(2/3)*d_sigma*dg1,...
                                             Jt+(2/3)*d_sigma*dJt1,...
                                             eta+(2/3)*d_sigma*deta1,...
                                             q,dq,Ba,shape,Nmode,L0,m,w,d,d_sigma);   
 
    eta =  eta + (d_sigma/4 * (deta1 + 3*deta2));                      
    g   =  g   + (d_sigma/4 * (dg1   + 3*dg2  ));
    Jt  =  Jt  + (d_sigma/4 * (dJt1  + 3*dJt2 ));
    M   =  M   + (d_sigma/4 * (dM1   + 3*dM2  )); 
    C   =  C   + (d_sigma/4 * (dC1   + 3*dC2  ));
    G   =  G   + (d_sigma/4 * (dG1   + 3*dG2  ));
    
end

