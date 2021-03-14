function [J] = funcJacobianopt(k,e,L0,space_step)
q = [k,e];       % modal coordinates (check order first curvature then elongation)

%% New implementation
Q0 = rot2quat(eye(3));   % initial no rotation 
r0 = zeros(3,1);          % position vector is 0
g0 = [Q0(:);r0];          % initial condition on g
g =g0;

J0 = zeros(6,2);       % start of with empty jacobian matrix
Jt = J0;

d_sigma = L0/space_step;
L = 0:d_sigma:L0;

for ii = 1:length(L)
    
    sigma = L(ii);
    [dg1,dJt1] = jacODEopt(sigma,g,q,L0);
    [dg2,dJt2] = jacODEopt(sigma+(2/3*d_sigma),g+(2/3)*d_sigma*dg1,q,L0);
    
    g  = g  + (d_sigma/4 * (dg1  + 3*dg2 ));
    Jt = Jt + (d_sigma/4 * (dJt1 + 3*dJt2));
    
   
end
% End effector Jacobian
J = adjointGInv(g(1:4),g(5:7))*Jt;




