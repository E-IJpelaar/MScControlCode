function q = cs2ts(x_set,y_set,theta_set,L0)


q0 = rand(2,1);
q = fminsearch(@(q) Ets(q,x_set,y_set,theta_set,L0),q0);
% q1 = kappa
% q2 = epsilon

end


function Eq = Ets(q,x_set,y_set,theta_set,L0)

r_opt = ts2cs3(q(1),q(2),L0);
e_theta = r_opt(1)-theta_set;
e_x = r_opt(2)- x_set;
e_y = r_opt(3)-y_set;


%scaling theta >> y >> x generally. Probably prioritize theta  then y and
% then x
a1 = 1;  % e theta
a2 = 1;  % e x
a3 = 1;% e y

Eq = norm(a1*e_theta + a2*e_x + a3*e_y);

end
