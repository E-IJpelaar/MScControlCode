function [deta,dg, dJt, dM,dC,dG] = massODEV2(sigma,g,Jt,eta,q,dq,Ba,shape,Nmode,L0,m,w,d,d_sigma)
Q = g(1:4);  % first 4 entries express rotation frame in quaternion
r = g(5:7);  % entries express position vector
gvec = [0;0;0;-9.81;0;0];
%% Calculate strain field
[xi,dxi] = strainFieldV2(sigma,q,dq,Ba,shape,Nmode,L0);  % get strain field 
%% first thing to look for errors: strainfield

K = xi(1:3);  % curvatures
E = xi(4:6);  % translations

%% Calculate Q' and r'
QK = quat2rotm(Q(:).')*K;             % intermediate calculation

AQK = [0    ,-QK(1),-QK(2),-QK(3);
       QK(1), 0    ,-QK(3), QK(2);
       QK(2), QK(3), 0    ,-QK(1);
       QK(3),-QK(2), QK(1), 0   ];    % see Boyer "forward dynamics of continuum and soft robots a strain parameterization based approach
     
dQdl = (2*norm(Q))\(AQK*Q);           % Q'
drdl = quat2rotm(Q(:).')*E;           % r'
dg = [dQdl(:);drdl(:)];

%% Jacobian 
Adg = adjointG(Q(:),r);
% BaPhi = [0,0;sigma,0;0,0;0,sigma;0,0;0,0];
[BaPhi]  = shapeValue(shape,Nmode,sigma,Ba,L0); % second thing to look at
dJt = Adg*BaPhi;         % J_tilde only the part inside the integral (factor 5? Adg*BaPhi*5)

%% Intermediate calculation
invAdg = adjointGInv(Q(:),r);  % Adg^-1
invAdgJt = invAdg*Jt;          % Adg^-1*J_tilde

[adxi] = adjointxi(xi);        % ad_xi
[adeta] = adjointeta(eta);     % ad_eta
deta = -adxi*eta + dxi;        % n' = -ad_xi eta + dxi;

%% Mass matrix
[m_sigma,J1,J2,J3] = inertiaRectangle(m,L0,w,d,d_sigma); 
Mdiag = diag([J1,J2,J3,m_sigma,m_sigma,m_sigma]);  % J is a factor 50/100 too low
dM = (invAdgJt)'*Mdiag*(invAdgJt);  %invAdgJt wrong calculated?
% approach get code brandon running, and compare dM invAdg etc etc.

%% Coreolis matrix
Jtdt = Adg*adeta*BaPhi; 
dC = (invAdgJt)'*(Mdiag*(invAdg*Jtdt) + (Mdiag*adeta*invAdgJt) - (adeta'*Mdiag*invAdgJt));
% C = J'*M*dJ + J'*M*ad_n*J - J'*ad_n'*M*J where J = inv(Ad_g)*J_tilde

%% Gravity vector
dG = invAdgJt'*Mdiag*invAdg*gvec;