function normF = stiffnessModel(x,e,F_eff)

F_exp = F_eff;
K = x(1) + x(2).*((tanh(x(3).*e)).^2 -1);
F = K.*e;


normF =  sum((F_exp - F).^2);

