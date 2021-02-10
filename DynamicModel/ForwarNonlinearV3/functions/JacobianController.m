function u = JacobianController(J,r,x)

Kp = 10;
Kd = 0;


e = r - x;
de = 0;

u = J'*(Kp*e + Kd*de);
