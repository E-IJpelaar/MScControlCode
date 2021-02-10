function dxdt = nonLinearDynamicModelV2(t,x,D,H,L0,m,w,d,Nmode,shape,space_step,corgrav)

k = x(1);                  % current actuator length
e = x(2);
dk = x(3);
de = x(4);


% Determine non-linear mass matrix
if corgrav ==1
    [M,C,G]  = massMatrixV2(k,e,dk,de,L0,m,w,d,Nmode,shape,space_step);
else
    [M]  = massMatrix(k,e,L0,m,w,d,Nmode,shape,space_step);
    C = zeros(2,2);
    G = zeros(2,1);
end

invM = inv(M);


% Calculate stiffness at instant x
Kk = bendStiffness(k);
Ke = elongStiffness(e);
K = diag([Kk,Ke]);        % Stiffness matrix

% Non-linear state-space
O2 = zeros(2,2);
o2 = zeros(2,1);
I2 = eye(2,2);

A = [ O2     , I2     ;
     -invM*K ,-invM*(C+D)];
 
B = [ O2;
      invM*H   ];
  
Gvec = [ o2 ;
         G ];

u = [10;0];
% u = [10;10]*smoothstep(t); 

dxdt = A*x + B*u - Gvec;

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
% end
