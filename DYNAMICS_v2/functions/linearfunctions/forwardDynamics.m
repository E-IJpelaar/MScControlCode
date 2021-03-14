function dxdt = forwardDynamics(t,x,A,B)

u = zeros(2,1);
u(1,1) = 60;
u(2,1) = 60;


dxdt = A*x + B*u;