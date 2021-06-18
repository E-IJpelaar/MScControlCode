function dxdt = nonLinearDynamicModelPressureModelParameterEstimation(t,x,D,H,L0,m,w,d,Nmode,shape,space_step,dt,Voltstep)
k = x(1);                  
e = x(2);
p1  = x(5);
p2  = x(6);

p = [p1;p2];
q = [k;e];

%% Pressure model dp/dt = -0.28p(t) + 2.68V(t) (old one 08-04)
% C_p1 = diag([0.285336645573371 , 0.285336645573371]);
% C_p2 = diag([2.687852328855003,2.687852328855003]);



% Updated pressure model for sinus input (10-06)
% C_p1 = diag([0.571455683571649 , 0.571455683571649]);
% C_p2 = diag([3.935359161537569 , 3.935359161537569]);

if Voltstep == 7
% % Updated pressure model for step 7V unequal pump characteristics
C_p1 = diag([0.355712155228543 , 0.390388153167751 ]);
C_p2 = diag([2.555977035533849 , 2.500231432247781 ]);
 
elseif Voltstep == 9
% % Updated pressure model for step 9V unequal pump characteristics
C_p1 = diag([0.391483771758633 , 0.412319874808244]);
C_p2 = diag([2.776372523300703 , 2.682276442785534]);

else

% Updated pressure model for step 11V unequal pump characteristics
C_p1 = diag([0.429331940258281 , 0.433505620264278]);
C_p2 = diag([2.908521842609676 , 2.767300264859135]);

    end



%% Determine Mass and Jacobian
[M,J]  = massMatrix(k,e,L0,m,w,d,Nmode,shape,space_step);
Jc = J(4:2:6,:);
invM = inv(M);


%% Stiffness matrix
% Calculate stiffness at instant x
Kk = bendStiffness(k);
Ke = elongStiffness(e);
% K = diag([30*Kk,4*Ke]);       % Stiffness matrix with mapped H
% K = diag([2.75*Kk,3.7*Ke]);        % Stiffness matrix with old H

% K = diag([3*Kk,3.5*Ke]);        % Best for 7V peak 1
% K = diag([3.5*Kk,3.5*Ke]);        % Best for 7V peak 2
% K = diag([3.25*Kk,3.5*Ke]);        % Best for 7V combined




% K = diag([3*Kk,3.5*Ke]);        % Best for 9V peak 1
% K = diag([3.25*Kk,3.5*Ke]);        % Best for 9V peak 2
% K = diag([3.125*Kk,3.5*Ke]);        % Best for 9V combined

% K = diag([3*Kk,3.5*Ke]);        % Best for 11V peak 1
% K = diag([3*Kk,3.5*Ke]);        % Best for 11V peak 2
% K = diag([3*Kk,3.5*Ke]);        % Best for 11V peak combined

K = diag([3.125*Kk,3*Ke]);        % OVERALL BEST

% Non-linear state-space
O2 = zeros(2,2);
I2 = eye(2,2);

%% 

if t < 2
    U(1) = 0;
    U(2) = 0;
elseif   t >= 2 && t < 22
   U(1) = Voltstep;
   U(2) = 0;
elseif   t>=22 && t < 62
   U(1) = 0;
   U(2)= 0;
elseif   t>=62 && t < 82
   U(1) = 0;
   U(2)= Voltstep;
elseif   t>=82 && t < 122
   U(1) = 0;
   U(2)= 0;
elseif   t>=122 && t < 142
   U(1) = Voltstep;
   U(2) = Voltstep;
else   
   U(1) = 0;
   U(2)= 0;
end

U = U'; 

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



