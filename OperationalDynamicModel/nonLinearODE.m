function [dx] = nonLinearODE(t,x,D,H,L0,m,w,d,Nmode,shape,space_step,r_ref)

k  = x(1);                  % current actuator length
e  = x(2);
dk = x(3);
de = x(4);

% Mass matrix
[M,J]  = massMatrix(k,e,L0,m,w,d,Nmode,shape,space_step);
Jc = J(2:2:6,:);
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
      invM*H];  % pressure control
  
%% without pressure effects  
% controller

r = ts2cs(k,e,L0);   % [kappa;epsilon]==>[theta(degrees); x;y]


error = r_ref-r;
derror = zeros(3,1);
% 
Kp = [100,   0   ,   0   ;
        0  , 10000 ,   0   ;
        0  ,   0   , 10000];
Kd = diag([2,5,5]);

tau = Jc.'*(Kp*error + Kd*derror);

p_ci = inv(H)*tau; % control input signal
 
p_max = 80;

if p_ci(1,1)> p_max 
    p_ci(1,1)= p_max;
elseif p_ci(1,1) < 0
    p_ci(1,1) = 0;
end
if p_ci(2,1)> p_max 
    p_ci(2,1)= p_max;
elseif p_ci(2,1) < 0
    p_ci(2,1) = 0;
end
%  
p = p_ci*smoothstep(t); % actual control input with saturation and actuator dynamics ()
% 
% figure(5)
% plot(t,p(1,1),'x')
% hold on; grid on;
% plot(t,p(2,1),'o')

% figure(6)
% plot(t,tau(1,1),'x')
% hold on; grid on;
% plot(t,tau(2,1),'o')

dx = A*x + B*p;


end


function y = smoothstep(X)

y = zeros(length(X),1); 

for ii = 1:length(X)
    x = X(ii);
    if x<=0, y(ii,1) = 0;
    elseif (x>0 && x<=1), y(ii,1) = 3*x.^2 -2*x.^3;
    else, y(ii,1) = 1;
    end
end
end
