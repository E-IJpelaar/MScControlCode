function dxdt = nonLinearDynamicModelV2(t,x,D,H,L0,m,w,d,Nmode,shape,space_step)

k = x(1);                  % current actuator length
e = x(2);


% Determine non-linear mass matrix
[M]  = massMatrixV2(k,e,L0,m,w,d,Nmode,shape,space_step)


% M = [ 0.0005  ,  0.0000;
%     0.0000 ,   0.4816]*1e-4;

invM = inv(M);

% Calculate stiffness at instant x
Kk = bendStiffness(k);
Ke = elongStiffness(e);
K = diag([Kk,Ke]);        % Stiffness matrix

% Non-linear state-space
O2 = zeros(2,2);
I2 = eye(2,2);

A = [ O2     , I2     ;
     -invM*K ,-invM*D];
 
B = [ O2;
      invM*H   ];

u = [0;0]*smoothstep(t); 

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
