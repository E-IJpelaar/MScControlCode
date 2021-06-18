function Ep = ePressurePWCadapted(x,t,p,V)

gamma = x(1);
delta = x(2);

alpha = x(3);
beta  = x(4);

tau = gamma.*V + delta;
K = alpha.*V + beta;

p_star = K.*(1-exp(-t./tau)).*V;
Ep = sum((p - p_star).^2);

















