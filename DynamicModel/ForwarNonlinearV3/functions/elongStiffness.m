function K_e = elongStiffness(x)

alpha = [1393.64242400988,1377.62344363938,0.278649068640955]; % Elognation stiffness parameters as determined
K_e  = (alpha(1) + alpha(2).*((tanh(alpha(3).*x)).^2 -1));