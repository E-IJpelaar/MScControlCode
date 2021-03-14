function Ep = ePressurePWC(x,t,p,V)

tau   = x(1);
alpha = x(2);
beta  = x(3);


K = alpha.*V + beta;
p_star = K.*(1-exp(-t./tau)).*V;
Ep = sum((p - p_star).^2);

















