function [Baphi_s] = shapeValue(shape,Nmode,l,Ba,L0)
%% Calculate strains and curvatures with shape functions
[n,m] = size(Ba);        % amount of active strains
l = l/L0;
Phi = zeros(n,Nmode*m);  % pre allocate
[row,~] = find(Ba == 1); 

if strcmp(shape,'cheby')        % chebyshev shape functions
    for ii = 1:length(row)
        phi=zeros(1,Nmode);
            for kk = 0:Nmode-1
                phi(1,kk+1) = cos(kk*acos(l));
            end 
        Phi(row(ii),ii*Nmode-Nmode+1:ii*Nmode)  = phi;
    end 

    
elseif strcmp(shape,"poly")       % polynomial shape functions
    for ii = 1:length(row)
        phi=zeros(1,Nmode);
        for kk = 0:Nmode-1
                phi(1,kk+1) = (l^kk);
        end
    Phi(row(ii),ii*Nmode-Nmode+1:ii*Nmode)  = phi;
    end



elseif strcmp(shape,"legendre")  % legengdre shape functions
    for ii = 1:length(row)
        phi=zeros(1,Nmode);
        for kk = 0:Nmode-1
            if kk == 0
            phi(1,kk+1) = 1;
            elseif kk == 1
            phi(1,kk+1) = l;  
            elseif kk == 2
            phi(1,kk+1) = 0.5*(3*l^2 - 1);
            elseif kk == 3
            phi(1,kk+1) = 0.5*(5*l^3 - 3*l);
            elseif kk == 4
            phi(1,kk+1) = 0.125*(35*l^4 - 30*l^2 + 3);
            elseif kk == 5
            phi(1,kk+1) = 0.125*(63*l^5 - 70*l^3 + 15*l);
            else
            error('legendre polynomials only defined untill degree == 5')
            end
        end
      Phi(row(ii),ii*Nmode-Nmode+1:ii*Nmode)  = phi;
    end

    
else
   error('This shape has not been defined')
end

Baphi_s = Phi;
