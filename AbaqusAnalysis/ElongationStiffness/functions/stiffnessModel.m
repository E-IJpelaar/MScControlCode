function normF = stiffnessModel(x,p_q2,F_mapped)

F_exp = F_mapped;
K = x(1) + x(2).*((tanh(x(3).*p_q2)).^2 -1);
F = K.*p_q2;


normF =  sum((F_exp - F).^2);

