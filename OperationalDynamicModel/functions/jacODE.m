function [dg, dJt] = jacODE(sigma,g,q,Ba,shape,Nmode,L0)


Q = g(1:4);  % first 4 entries express rotation frame in quaternion
r = g(5:7);  % entries express position vector


%% Calculate strain field

[xi] = strainField(sigma,q,Ba,shape,Nmode,L0);  % get strain field

K = xi(1:3);  % curvatures
T = xi(4:6);  % translations

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

end