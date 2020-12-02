function dxdt = linmodel(t,x,A,B)
x = [x(1);x(2)];
u = [100;10];
dxdt = A*x + B*u;



