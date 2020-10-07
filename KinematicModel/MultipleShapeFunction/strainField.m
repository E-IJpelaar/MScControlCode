function [xi] = strainField(l,q,Ba,shape,Nmode)
%% Initial undeformed shape
xi0 = [0;0;0;0;0;1];

%% Calculate strains and curvatures with shape functions
[~,m] = size(Ba);        % amount of active strains
phi_q = zeros(m,1);      % shape function vector (phi*q)

if shape == "cheby"      % chebyshev shape functions
    for ii = 1:m
        q_mode = q(1+(ii-1)*Nmode:ii*Nmode);
        for n = 0:Nmode-1
            phi_q(ii,1) = phi_q(ii,1) + (cos(n*acos(l))*q_mode(n+1));
        end
    end
end

if shape == "poly"       % polynomial shape functions
    for ii = 1:m
        q_mode = q(1+(ii-1)*Nmode:ii*Nmode);
        for n = 0:Nmode-1
            phi_q(ii,1) = phi_q(ii,1) + (l^n)*q_mode(n+1);
        end
    end
end

if shape == "legendre"   % legendre shape functions
    for ii = 1:m
        q_mode = q(1+(ii-1)*Nmode:ii*Nmode);
        for n = 0:Nmode-1
            if n == 0
            phi_q(ii,1) = phi_q(ii,1) + 1*q_mode(n+1);
            elseif n == 1
            phi_q(ii,1) = phi_q(ii,1) + l*q_mode(n+1);  
            elseif n == 2
            phi_q(ii,1) = phi_q(ii,1) + 0.5*(3*l^2 - 1)*q_mode(n+1);
            elseif n == 3
            phi_q(ii,1) = phi_q(ii,1) + 0.5*(5*l^3 - 3*l)*q_mode(n+1);
            elseif n == 4
            phi_q(ii,1) = phi_q(ii,1) + 0.125*(35*l^4 - 30*l^2 + 3)*q_mode(n+1);
            elseif n == 5
            phi_q(ii,1) = phi_q(ii,1) + 0.125*(63*l^5 - 70*l^3 + 15*l)*q_mode(n+1);
            else
            error('legendre polynomials only defined untill degree == 5')
            end
        end
    end
end

%% Determine strain and curvature vector and matrix
xi = Ba*phi_q + xi0;    % determine xi vector
