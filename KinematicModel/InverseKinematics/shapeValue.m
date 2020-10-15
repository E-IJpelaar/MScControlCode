function [Baphi_s] = shapeValue(shape,Nmode,l,Ba)
%% Calculate strains and curvatures with shape functions
[n,m] = size(Ba);        % amount of active strains

Phi = zeros(n,Nmode*m);  % pre allocate
[row,~] = find(Ba == 1); 

if shape == 'cheby'
    for ii = 1:length(row)
        phi=zeros(1,Nmode);
        for kk = 0:Nmode-1
                phi(1,kk+1) = cos(kk*acos(l));
        end 
    Phi(row(ii),ii*Nmode-Nmode+1:ii*Nmode)  = phi;
    end 
else 
    error('This shape has not been defined')
end

Baphi_s = Phi;
