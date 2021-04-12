function dxdt = nonLinearDynamicModelPressureModel(t,x,D,H,L0,m,w,d,Nmode,shape,space_step,r_ref,dt,Kp,Ki,Kd)
global tau_error_sum p_error_sum U1 U2 time pset1 pset2 tau1 tau2;
k = x(1);                  
e = x(2);
p1  = x(5);
p2  = x(6);



p = [p1;p2];
q = [k;e];

K_poly1 = [  -0.0014 ;   0.0578 ; -0.8828 ;   5.8426 ;  -7.2264];
K_poly2 = [  -0.0014 ;   0.0578 ; -0.8828 ;   5.8426 ;  -7.2264];

% Nonlin set-up
[M,J]  = massMatrix(k,e,L0,m,w,d,Nmode,shape,space_step);
Jc = J(4:2:6,:);
invM = inv(M);


% Calculate stiffness at instant x
Kk = bendStiffness(k);
Ke = elongStiffness(e);
K = diag([Kk,Ke]);        % Stiffness matrix

% Non-linear state-space
O2 = zeros(2,2);
I2 = eye(2,2);


 
%% Jacobian controller
r = ts2cs(k,e,L0);   % [kappa;epsilon]==>[theta(degrees); x;y]

error = r_ref-r;
tau_error_sum = tau_error_sum + error*dt;
derror = [0;0] - Jc*q;


% Kd = diag([0,0]);

Kpt = Kp*error;
Kit = Ki*tau_error_sum;
Kdt = Kd*derror;


k_set = 10;
e_set = 0.3;
q_set = [k_set;e_set];
Kk_set = bendStiffness(k_set);
Ke_set = elongStiffness(e_set);
K_set = diag([Kk_set,Ke_set]); 


tau = Jc'*(Kpt + Kit + Kdt);% + K_set*[q_set];%  + Kdt);


V = [27,2.5;-27,2.5]*tau;
% %% Pressure controller
p_set = inv(H)*tau;

% p_error = p_set - p
% p_error_sum = p_error_sum + p_error*dt;



% Pp = Kpp*p_error;
% Ip = Kip*p_error_sum;


% Pressure input Volt
% V = Pp + Ip;

% saturate between 0 12V input



U = min(max(V,0),12);

if t == 0
    ii = 1;
    
U1(1,ii) = U(1);
U2(1,ii) = U(2);    
time(1,ii) = t;   
pset1(1,ii) = p_set(1);
pset2(1,ii) = p_set(2);
tau1(1,ii) = tau(1);
tau2(1,ii) = tau(2);

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
    

end





% input dependent pump model
K1 = polyval(K_poly1,U(1,1));
K2 = polyval(K_poly2,U(2,1));

if K1 <= 0
    K1 =0;
end
if K2 <= 0
    K2 =0;
end


tau_s1 = -0.1727*U(1,1) +  4.2069;
tau_s2 = -0.1727*U(2,1) +  4.2069;

C_p1 = diag([1/tau_s1 ,1/tau_s2]);

% determine dynamics matrix
A = [ O2     , I2     , O2  ;
     -invM*K ,-invM*D , O2  ;
     O2     ,  O2    , -C_p1];
 
 
C_p2 = diag([K1/tau_s1,K2/tau_s2]);


% determine input matrix
B = [  O2  ,  O2 ;
      invM*H ,  O2 ;
       O2  ,  C_p2 ];  % jacobian control

% input vector
u = [ p;
      U];
  


dxdt = A*x + B*u;
end



