
function dxdt = pressureODEPWC(t,x,mu,nu,alpha,beta,V,Vmax)

tau = mu*V + nu;

if  V <= Vmax 
        K = alpha*V + beta;
elseif V > Vmax
        K = alpha*Vmax +beta;
end


dxdt = (-1/tau) * x + (K/tau) * V;

end


