function eta =strainField(t,y,Ba)

q = [0.5;0.5];   % q is a function of t, it is taken constant for all time t.
Phi = eye(2);    % only one shape function for 2 dof, independent of sigma.
                 % so, eta = constant for all sigma and t
eta = Phi*q;



