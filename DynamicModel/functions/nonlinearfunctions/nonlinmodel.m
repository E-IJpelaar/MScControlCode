function [dxdt] = nonlinmodel(t,x,invD,H)
x = [x(1);x(2)];

% p = inv(H)*(inv(D)*dxdt + K*x)
% p1 = pressure1(t);
% p2 = pressure2(t);

p1 = 0;%75+25.*sin(2.*pi.*0.1.*t);
p2 = 10;%75-25.*sin(2.*pi.*0.1.*t);

p = [p1;p2];

K_e = elongStiffness(x(1)); % [N] non-linear elongation stiffness
K_k = bendStiffness(x(2));  % [Nm^2] non-linear bending stiffness

K = diag([K_e,K_k]);        % Stiffness matrix

dxdt= invD*(H*p - K*x);