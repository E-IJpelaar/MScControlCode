function [xi,dxi] = strainFieldV2(sigma,q,dq,Ba,shape,Nmode,L0)
%% Initial undeformed shape
xi0 = [0;0;0;1;0;0];
%% Calculate strains and curvatures with shape functions
[~,m] = size(Ba);        % amount of active strains
phi_q = zeros(m,1);      % shape function vector (phi*q)
phi_dq = zeros(m,1);     % shape function vector (phi*q)
l = sigma/L0;            % normalize shape function to 1
if shape == "cheby"      % chebyshev shape functions
    for ii = 1:m
        q_mode  = q(1+(ii-1)*Nmode:ii*Nmode);
        dq_mode = dq(1+(ii-1)*Nmode:ii*Nmode);
        for n = 0:Nmode-1
            phi_q(ii,1)  = phi_q(ii,1)  + (cos(n*acos(l))*q_mode(n+1));
            phi_dq(ii,1) = phi_dq(ii,1) + (cos(n*acos(l))*dq_mode(n+1));
        end
    end
end

if shape == "poly"       % polynomial shape functions
    for ii = 1:m
        q_mode   = q(1+(ii-1)*Nmode:ii*Nmode);
        dq_mode  = dq(1+(ii-1)*Nmode:ii*Nmode);
        for n = 0:Nmode-1
            phi_q(ii,1)  = phi_q(ii,1)  + (l^n)*q_mode(n+1);
            phi_dq(ii,1) = phi_dq(ii,1) + (l^n)*dq_mode(n+1);
        end
    end
end

if shape == "legendre"  % legendre shape functions
    for ii = 1:m
        q_mode = q(1+(ii-1)*Nmode:ii*Nmode);
        dq_mode = dq(1+(ii-1)*Nmode:ii*Nmode);
        for n = 0:Nmode-1
            if n == 0
            phi_q(ii,1) = phi_q(ii,1) + 1*q_mode(n+1);
            phi_dq(ii,1) = phi_dq(ii,1) + 1*dq_mode(n+1);
            elseif n == 1
            phi_q(ii,1) = phi_q(ii,1) + l*q_mode(n+1); 
            phi_dq(ii,1) = phi_dq(ii,1) + l*dq_mode(n+1); 
            elseif n == 2
            phi_q(ii,1) = phi_q(ii,1) + 0.5*(3*l^2 - 1)*q_mode(n+1);
            phi_dq(ii,1) = phi_dq(ii,1) + 0.5*(3*l^2 - 1)*dq_mode(n+1);
            elseif n == 3
            phi_q(ii,1) = phi_q(ii,1) + 0.5*(5*l^3 - 3*l)*q_mode(n+1);
            phi_dq(ii,1) = phi_dq(ii,1) + 0.5*(5*l^3 - 3*l)*dq_mode(n+1);
            elseif n == 4
            phi_q(ii,1) = phi_q(ii,1) + 0.125*(35*l^4 - 30*l^2 + 3)*q_mode(n+1);
            phi_dq(ii,1) = phi_dq(ii,1) + 0.125*(35*l^4 - 30*l^2 + 3)*dq_mode(n+1);
            elseif n == 5
            phi_q(ii,1) = phi_q(ii,1) + 0.125*(63*l^5 - 70*l^3 + 15*l)*q_mode(n+1);
            phi_dq(ii,1) = phi_dq(ii,1) + 0.125*(63*l^5 - 70*l^3 + 15*l)*dq_mode(n+1);
            else
            error('legendre polynomials only defined untill degree == 5')
            end
        end
    end
end

%% Determine strain and curvature vector and matrix
xi = Ba*phi_q + xi0;              % determine xi vector
xi(1:3) = xi(1:3)*norm(xi(4:6));  % account for curvature due to elongation
dxi = Ba*phi_dq;
% dxi(1:3) = dxi(1:3)*norm(dxi(4:6));  % account for curvature due to elongation
