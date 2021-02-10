function [dg, dJt, dM] = massODE2(sigma,g,Jt,q,Ba,shape,Nmode,L0,m,w,d,ds,J_cor)


Q = g(1:4);  % first 4 entries express rotation frame in quaternion
r = g(5:7);  % entries express position vector


%% Calculate strain field

[xi] = strainField2(sigma,q,Ba,shape,Nmode,L0);  % get strain field

T = xi(1:3);  % translations
K = xi(4:6);  % curvatures


%% Calculate Q' and r'
QK = quat2rotm(Q(:).')*K;             % intermediate calculation

AQK = [0    ,-QK(1),-QK(2),-QK(3);
       QK(1), 0    ,-QK(3), QK(2);
       QK(2), QK(3), 0    ,-QK(1);
       QK(3),-QK(2), QK(1), 0   ];    % see Boyer "forward dynamics of continuum and soft robots a strain parameterization based approach
     
dQdl = (2*norm(Q))\(AQK*Q);           % Q'
drdl = quat2rotm(Q(:).')*T;           % r'

dg = [dQdl(:);drdl(:)];

%% Jacobian 
Adg = adjointG(Q(:),r);
[BaPhi]  = shapeValue(shape,Nmode,sigma,Ba,L0); 

dJt = Adg*BaPhi;         % J_tilde only the part inside the integral

%% Mass matrix
invAdg = adjointGInv2(Q(:),r);
[m_sigma,J1,J2,J3] = inertiaRectangle(m,L0,w,d,ds,J_cor); 
Mdiag = diag([m_sigma,m_sigma,m_sigma,J1,J2,J3]);  
dM = (invAdg*Jt)'*Mdiag*(invAdg*Jt);

end