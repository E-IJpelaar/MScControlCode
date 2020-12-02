function K_k = bendStiffness(x)

beta = [2300.86746927188,2300.84669008190,0.00741771829813126]; % Bending stiffness parameters as determined
K_k  = (beta(1) + beta(2).*((tanh(beta(3).*x)).^2 -1));