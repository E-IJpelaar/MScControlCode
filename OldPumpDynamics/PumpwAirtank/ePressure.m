function Ep = ePressure(x,t,p,V)

tau   = x(1);
K     = x(2);

p_star = K.*(1-exp(-t./tau)).*V;
Ep = sum((p - p_star).^2);


