function K = funcK(V,alpha,beta,Vmax)

    if  V <= Vmax 
        K = alpha.*V + beta;
    elseif V > Vmax
        K = alpha.*Vmax +beta;
    end




end