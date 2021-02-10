function dVdl = KinematicsJacobian(l,V,q,Ba,shape,Nmode,L0)

Q = V(1:4);  % first 4 entries express rotation frame in quaternion
r = V(5:7);  % entries express position vector
J(:,1) = V(8:13); % entries express Jacobian
J(:,2) = V(14:19);% entries express Jacobian
%% Calculate strain field

[xi] = strainField(l,q,Ba,shape,Nmode,L0);  % get strain field

K = xi(1:3);  % curvatures
T = xi(4:6);  % translations

%% Calculate Q' and r'
QK = quat2rotm(Q(:).')*K;  % intermediate calculation

AQK = [0    ,-QK(1),-QK(2),-QK(3);
       QK(1), 0    ,-QK(3), QK(2);
       QK(2), QK(3), 0    ,-QK(1);
       QK(3),-QK(2), QK(1), 0   ];    % see Boyer "forward dynamics of continuum and soft robots a strain parameterization based approach
     
dQdl = (2*norm(Q))\(AQK*Q);           % Q'
drdl = quat2rotm(Q(:).')*T;           % r'

%% Calculate Integral (Adg Ba Phi(sigma))
[BaPhi]  = shapeValue(shape,Nmode,l,Ba,L0); 
Adg = adjointG(Q,r);
dJdl = Adg*BaPhi;

dVdl = [dQdl;drdl;dJdl(:)];
