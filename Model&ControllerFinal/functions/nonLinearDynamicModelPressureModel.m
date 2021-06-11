function dxdt = nonLinearDynamicModelPressureModel(t,x,D,H,L0,m,w,d,Nmode,shape,space_step,r_ref,dt,Kp,Ki,Kpp,Kip)
global tau_error_sum p_error_sum U1 U2 time pset1 pset2 tau1 tau2 TAU1 TAU2 prev_TAU filJ;

k = x(1);                  
e = x(2);
p1  = x(5);
p2  = x(6);

p = [p1;p2];
q = [k;e];

%% Pressure model dp/dt = -0.28p(t) + 2.68V(t)
C_p1 = diag([0.285336645573371 , 0.285336645573371]);
C_p2 = diag([2.687852328855003,2.687852328855003]);

%% Determine Mass and Jacobian
[M,J]  = massMatrix(k,e,L0,m,w,d,Nmode,shape,space_step);
Jc = J(4:2:6,:);
invM = inv(M);


%% Stiffness matrix
% Calculate stiffness at instant x
Kk = bendStiffness(k);
Ke = elongStiffness(e);
K = diag([100*Kk,4*Ke]);        % Stiffness matrix

% Non-linear state-space
O2 = zeros(2,2);
I2 = eye(2,2);


 
%% Jacobian controller
r = ts2cs(k,e,L0);   % [kappa;epsilon]==>[theta(degrees); x;y]

error = r_ref-r;
tau_error_sum = tau_error_sum + error*dt;


Kpt = Kp*error;
Kit = Ki*tau_error_sum;

tau = Jc'*(Kpt + Kit);


TAU = filJ*tau + (1-filJ)*prev_TAU;





%% Pressure controller
p_set = inv(H)*TAU;
% p_set = inv(H)*tau;
p_error = p_set - p;
p_error_sum = p_error_sum + p_error*dt;

Pp = Kpp*p_error;
Ip = Kip*p_error_sum;

% Pressure input Volt
V = Pp + Ip;
U = min(max(V,0),12);


%% Log data
if t == 0
    ii = 1;
    
U1(1,ii) = U(1);
U2(1,ii) = U(2);    
time(1,ii) = t;   
pset1(1,ii) = p_set(1);
pset2(1,ii) = p_set(2);
tau1(1,ii) = tau(1);
tau2(1,ii) = tau(2);

TAU1(1,ii) = TAU(1);
TAU2(1,ii) = TAU(2);
prev_TAU  = TAU;

% elseif mod(t,dt) < 9e-6
elseif mod(t,dt) < 9e-5
ii = round(t/dt)+1;

U1(1,ii) = U(1);
U2(1,ii) = U(2);
time(1,ii) = t;  
pset1(1,ii) = p_set(1);
pset2(1,ii) = p_set(2);
tau1(1,ii) = tau(1);
tau2(1,ii) = tau(2);

TAU1(1,ii) = TAU(1);
TAU2(1,ii) = TAU(2);
prev_TAU  = TAU;
    

end


%% State-space update
% dynamics matrix
A = [ O2     , I2     , O2  ;
     -invM*K ,-invM*D , O2  ;
     O2     ,  O2    , -C_p1];

% input matrix
B = [  O2  ,  O2 ;
      invM*H ,  O2 ;
       O2  ,  C_p2 ];  % jacobian control

% input vector
u = [ p;
      U];
 
dxdt = A*x + B*u;

end



