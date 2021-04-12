function dxdt = nonLinearDynamicModelV2(t,x,D,H,L0,m,w,d,Nmode,shape,space_step,r_ref,dt)
global prev_error;
k = x(1);                  % current actuator length
e = x(2);
dk = x(3);
de = x(4);

dq = [dk;de];

% Mass matrix
[M,~]  = massMatrix(k,e,L0,m,w,d,Nmode,shape,space_step);
Jc = [0.0028,0;0,0.0677];
% Jc = J(4:2:6,:);
% Jc = Jc'*inv(Jc*Jc' + 0.001*eye(3));
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
      invM];  % jacobian control
  
%% without pressure effects  
% controller

r = ts2cs(k,e,L0);   % [kappa;epsilon]==>[theta(degrees); x;y]

edot = Jc*dq;
derror = edot;
error = r_ref-r;
% derror = (error - prev_error)/dt;
prev_error = error;


% 
% Kp = diag([60000,60000]);
% Kd = diag([5000,5000]);

Kp = diag([100,100]);  % works for this gains
Kd = diag([25,25]);    % works for this gains
tau = Jc'*(Kp*error + Kd*derror)


% Kp = diag([750,750]);
% Kd = diag([150,150]);
% tau = Kp*error + Kd*derror;
% tau = [0;0];
% p_ci = inv(H)*tau; % control input signal
 

dxdt = A*x + B*tau;




end

