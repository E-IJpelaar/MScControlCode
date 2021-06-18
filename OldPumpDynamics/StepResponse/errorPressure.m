function Enorm = errorPressure(x,t,p,u)


K = x(1);   %   Maximum
tau = x(2); % tijdsconstatne
Enorm = sum((p - (u.*K.*(1-exp(-t./tau)))).^2);