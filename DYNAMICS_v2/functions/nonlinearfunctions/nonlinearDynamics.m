function dxdt = nonlinearDynamics(t,x,invM,D,H)

% Calculate stiffness at instant x
Ke = elongStiffness(x(1));
Kk = bendStiffness(x(2));
K = diag([Ke,Kk]);        % Stiffness matrix

% Non-linear state-space
O2 = zeros(2,2);
I2 = eye(2,2);

A = [ O2     , I2     ;
     -invM*K ,-invM*D];
 
B = [ O2;
      invM*H   ];


u = zeros(2,1);

u(1,1) = 5;
u(2,1) = 0;
   

dxdt = A*x + B*u;