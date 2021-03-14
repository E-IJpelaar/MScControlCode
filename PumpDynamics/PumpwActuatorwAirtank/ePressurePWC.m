function Ep = ePressurePWC(x,t,p,V)

tau   = x(1);
alpha = x(2);
beta  = x(3);
Vmax =  x(4);

K = zeros(length(V),1);
p_star = zeros(length(V),1);


for ii = 1:length(V)
    
    if  V(ii) <= Vmax 
        K(ii) = alpha*V(ii) + beta;
    elseif V(ii) > Vmax
        K(ii) = alpha*Vmax +beta;
    end
    
   
    p_star(ii) = K(ii).*(1-exp(-t(ii)./tau)).*V(ii);
end

Ep = sum((p - p_star).^2);

















