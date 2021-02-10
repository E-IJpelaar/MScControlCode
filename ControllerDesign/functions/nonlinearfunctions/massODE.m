function [dg, dJt, dM] = massODE(sigma,g,Jt,q,Ba,shape,Nmode,L0,m,w,d,d_sigma)
Q = g(1:4);  % first 4 entries express rotation frame in quaternion
r = g(5:7);  % entries express position vector

%% Calculate strain field
[xi] = strainField(sigma,q,Ba,shape,Nmode,L0);  % get strain field

K = xi(1:3);  % curvatures
E = xi(4:6);  % translations

%% Calculate Q' and r'
QK = quat2rot(Q(:).')*K;             % R(Q)K

AQK = [0    ,-QK(1),-QK(2),-QK(3);   % A(K) == A(R(Q)K)
       QK(1), 0    ,-QK(3), QK(2);
       QK(2), QK(3), 0    ,-QK(1);
       QK(3),-QK(2), QK(1), 0   ];    % see Boyer "forward dynamics of continuum and soft robots a strain parameterization based approach
     
dQdl = (2*norm(Q))\(AQK*Q);           % Q'
drdl = quat2rotm(Q(:).')*E;           % r'
dg = [dQdl(:);drdl(:)];

%% Jacobian 
Adg = adjointG(Q(:),r);
[BaPhi]  = shapeValue(shape,Nmode,sigma,Ba,L0); %%
dJt = Adg*BaPhi;         % J_tilde only the part inside the integral

%% Mass matrix
invAdg = adjointGInv(Q(:),r);
[m_sigma,J1,J2,J3] = inertiaRectangle(m,L0,w,d,d_sigma); 
Mdiag = diag([J1,J2,J3,m_sigma,m_sigma,m_sigma]);  
dM = (invAdg*Jt)'*Mdiag*(invAdg*Jt);
