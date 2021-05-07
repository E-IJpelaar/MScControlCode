function K_k = bendStiffness(x)

beta = [3.03222658517135,3.03091196274452,0.00337548404982915]; % Bending stiffness parameters as determined
K_k  = (beta(1) + beta(2).*((tanh(beta(3).*x)).^2 -1));