function K_e = elongStiffness(x)

alpha = [1157.94570091901,1142.30596555048,0.313377131069976]; % Elognation stiffness parameters as determined
K_e  = (alpha(1) + alpha(2).*((tanh(alpha(3).*x)).^2 -1));