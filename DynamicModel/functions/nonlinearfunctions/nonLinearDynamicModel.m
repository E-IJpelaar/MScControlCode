function dxdt = nonLinearDynamicModelV2(t,x,D,H,L0,m,w,d,Nmode,shape,space_step)

e = x(1);
k = x(2);                  % current actuator length

% Determine non-linear mass matrix
[M]  = massMatrix(e,k,L0,m,w,d,Nmode,shape,space_step)
invM = inv(M);



% Calculate stiffness at instant x
Ke = elongStiffness(e);
Kk = bendStiffness(k);
K = diag([Ke,Kk]);        % Stiffness matrix

% Non-linear state-space
O2 = zeros(2,2);
I2 = eye(2,2);

A = [ O2     , I2     ;
     -invM*K ,-invM*D];
 
B = [ O2;
      invM*H   ];


u = [0;0]*smoothstep(t); 
  
% u = [20;0]*smoothstep(t);

dxdt = A*x + B*u;
end

% function y = smoothstep(X)
% 
% y = zeros(length(X),1); 
% 
% for ii = 1:length(X)
%     x = X(ii);
%     if x<=0, y(ii,1) = 0;
%     elseif (x>0 && x<=1), y(ii,1) = 3*x.^2 -2*x.^3;
%     else, y(ii,1) = 1;
%     end
% end
% 
%  
% 
% end
