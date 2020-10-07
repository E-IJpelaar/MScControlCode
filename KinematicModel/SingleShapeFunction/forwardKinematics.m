function dydl = forwardKinematics(l,g,q,Ba)

[xi_hat] = strainField(l,g,q,Ba);

g = reshape(g,4,4);

dydl = g*xi_hat;
dydl = reshape(dydl,16,1);