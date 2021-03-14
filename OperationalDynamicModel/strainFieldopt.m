function [xi] = strainFieldopt(l,q,L0)
%% Initial undeformed shape
xi0 = [0;0;0;0;0;1];
%% Calculate strains and curvatures with shape functions

Ba = [0,0;
      1,0;
      0,0;
      0,0;
      0,0;
      0,1];

m = 2;
Nmode = 1;
phi_q = zeros(m,1);      % shape function vector (phi*q)
l = l/L0;
   % chebyshev shape functions
    for ii = 1:m
        q_mode = q(1+(ii-1)*Nmode:ii*Nmode);
        for n = 0:Nmode-1
            phi_q(ii,1) = phi_q(ii,1) + (cos(n*acos(l))*q_mode(n+1));
        end
    end
%% Determine strain and curvature vector and matrix
xi = Ba*phi_q + xi0;    % determine xi vector
xi(1:3) = xi(1:3)*norm(xi(4:6));  % account for curvature due to elongation

