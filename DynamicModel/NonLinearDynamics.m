function dxdt = NonLinearDynamics(t,x,invM,D,H)

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


u = zeros(1,2);

u(1,1) = 10;
u(1,2) = 10;
    



dxdt = A*x + B*u';