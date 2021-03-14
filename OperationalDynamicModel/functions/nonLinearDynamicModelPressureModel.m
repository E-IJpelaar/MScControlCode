function dxdt = nonLinearDynamicModelPressureModel(t,x,D,H,L0,m,w,d,Nmode,shape,space_step,r_ref,dt)
global tau_error_sum p_error_sum;
k = x(1);                  % current actuator length
e = x(2);
% dk = x(3);
% de = x(4);
p1  = x(5);
p2  = x(6);

p = [p1;p2];
q = [k;e];

% w/o airtank
% tau_s = 0.095;
% K1 = 11.2;
% K2 = 11.2;

% w/ airtank w/actuator
tau_s = 3.8534;



% Nonlin set-up
[M,J]  = massMatrix(k,e,L0,m,w,d,Nmode,shape,space_step);
% Jc = J(4:2:6,:);

% Linear set-up
% M = diag([0.0005e-4,0.5330e-4]);
% Jc = diag([0.0023,0.0667]);

% Lin Mass non-lin Jac
M = diag([0.0005e-4,0.5330e-4]);
J = funcJacobianopt(k,e,L0,space_step);
Jc = J(4:2:6,:);



invM = inv(M);


% Calculate stiffness at instant x
Kk = bendStiffness(k);
Ke = elongStiffness(e);
K = diag([Kk,Ke]);        % Stiffness matrix

% Non-linear state-space
O2 = zeros(2,2);
I2 = eye(2,2);

C_p1 = diag([1/tau_s ,1/tau_s]);


A = [ O2     , I2     ,O2;
     -invM*K ,-invM*D ,O2;
     O2     ,  O2    , -C_p1];
 
 
%% Jacobian controller
r = ts2cs(k,e,L0);   % [kappa;epsilon]==>[theta(degrees); x;y]

error = r_ref-r;
tau_error_sum = tau_error_sum + error*dt;
derror = Jc*q;
 

% Kp = diag([80000,80000]); %% perfect for constant mass an Jacobian matrix
% Kd = diag([4000,4000]); %% M = diag([0.0005e-4,0.5330e-4])Jc = diag([0.0023,0.0667]);
%% tune procedure
% 1) tune Kp2 for pure [y] elongation first (you can't go to hight, limiting
% factor are the pumps) check what tau(2)[N] is needed 
% 2) Ki2 for pure elongation. (it should be relativley low, maybe add some anti-windup tools to improve) 
% 3) Kd2 not necessary. High stiffness, low mass very high oscillatory that are not interesting to damp
% 4) set [x,y] setpoint, tune Kp1. Probably Kp2 should be decreased. Best is to keep Kp1 and Kp2 close. Otherwise, one will cancel out the otherone and we get a system in which first y (or x) is positioned before x (or y) 
% as an effect Ki2 should be increased
% 5) Ki1 can be tuned, depending on x/angle of set point, Ki1 should be high/low comparable order as Ki2 (IDEA set point dependent controller)
% 6) Kd1 is actully not necessary. ratio bending stiffness/moment inertia
% is even higher, causing even higher freq osscilations.

% Kp = diag([10000,10000]);
% Ki = diag([40,475]);
% Kd = diag([0,0]);

Kp = diag([90000,90000]);
Ki = diag([50,325]);
Kd = diag([0,0]);

Kpt = Kp*error;
Kit = Ki*tau_error_sum;
Kdt = Kd*derror;


tau = Jc'*(Kpt + Kit  + Kdt);
% 
% 
% %% Pressure controller
p_set = inv(H)*tau;
p_error = p_set - p;
p_error_sum = p_error_sum + p_error*dt;



Kpp = diag([20,20]);
Kip = diag([0,0]);

Pp = Kpp*p_error;
Ip = Kip*p_error_sum;


% Pressure PI
V = Pp + Ip;



U = min(max(V,0),12);


K1 = PWC(U(1,1));
K2 = PWC(U(2,1));



C_p2 = diag([K1/tau_s,K2/tau_s]);


B = [  O2  ,  O2 ;
      invM ,  O2 ;
       O2  ,  C_p2 ];  % jacobian control

% input vector
u = [H*p;
      U];



dxdt = A*x + B*u;

end



