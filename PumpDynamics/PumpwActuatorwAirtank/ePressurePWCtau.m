function Ep = ePressurePWCtau(x,T,p,V)


mu = x(1);
nu = x(2);
alpha = x(3);
beta  = x(4);
Vmax = x(5);

tau = zeros(length(V),1);
K = zeros(length(V),1);
p_star = zeros(length(V),1);




for ii = 1:length(V)
    
    
    
    if  V(ii) <= Vmax 
        K(ii) = alpha*V(ii) + beta;
    elseif V(ii) > Vmax
        K(ii) = alpha*Vmax +beta;
    end

    
%     p_star(ii) = K(ii).*(1-exp(-t(ii)./tau(ii))).*V(ii);
    p_KV(ii) = K(ii)*V(ii);
    
end


for ii = 1:length(V)
    
   t = T(ii);
   tau(ii) = mu*V(ii) + nu;
   
   
    p_tau(ii) = (1-exp(-t./tau(ii)));
end
 
p_star = p_KV.*p_tau;

% K = alpha.*V + beta;

Ep = sum((p - p_star(:)).^2);

















