function dxdt = ForwardDynamics(t,x,A,B)

u = zeros(1,2);
u(1,1) = 50;%+25*sin(2*pi*0.01);
u(1,2) = 50;%-25*sin(2*pi*0.01);


dxdt = A*x + B*u';