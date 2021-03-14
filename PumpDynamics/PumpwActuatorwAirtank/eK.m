function errorK = eK(x,Vsteps,K_approx)

alpha = x(1);
beta = x(2);
Vmax = x(3);

for ii = 1:length(Vsteps)
    
    V= Vsteps(ii);
    
    if  V < Vmax 
        K_star(ii) = alpha.*V + beta;
    elseif V >= Vmax
        K_star(ii) = alpha.*Vmax +beta;
    end
    
    
    
end

errorK = sum((K_approx - K_star).^2);

end


