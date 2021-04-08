function [E]= errorFunction(x,z_ds,y_ds,z_mid_avgs,y_mid_avgs,Nmode,shape,L0)

q = x;                                             % set q equal to optimized variable x
[x_fkin,z_fkin] = funcKinematicsXZ(q,L0,Nmode,shape); % based on q determine [x-z] based on forward kinematics

% below the coordinates from the forward kinematic model are compared with
% the data set that is to be fit. Since the forward model gives a well
% discritized output, while only a limited data set is provided, indices
% need to found of the coordinates that match best. Namely, in the objective 
% function both vectors need to be of equal size.

p = zeros(1,length(z_mid_avgs));                   % pre-allocate
for ii = 1:length(z_mid_avgs)                      
    [~,p(ii)] = min((x_fkin-z_mid_avgs(ii)).^2 + (z_fkin-y_mid_avgs(ii)).^2); 
end

x_q = x_fkin(p);   % x - coordinates of the forward model that fit best to data
z_q = z_fkin(p);   % z - coordinates of the forward model that fit best to data

arclen = arclength(x_fkin,z_fkin,'s');               % arc-length of the forward kinematic model
arclen_mean = arclength(z_mid_avgs,y_mid_avgs,'s');  % indicated arc-length based on data set

%% Objective function
% term 1: error in position first coordiante to be minimized
% term 2: error in position second coordiante to be minimized
% term 3: error in arclength between data and forward model to be minimized
% term 4: error in position first end-effector coordinate to be minimized
% term 5: error in position second end-effector coordinate to be minimized
% E = sum((z_mid_avgs - x_q).^2 + (y_mid_avgs-z_q).^2 + (arclen_mean-arclen).^2+ (z_ds-x_fkin(end)).^2 + (y_ds-z_fkin(end)).^2 );
% term 6: minimize potential energy q'Qq

%% Optimized coefficients order of tuning tuned for Nmode = 2
% a1 = 39; 1
% a2 = 39; 2
% a3 = 35; 5
% a4 = 9; 4
% a5 = 9; 3
% Q  = 0*eye(Nmode*2); %E = 0.0250;

% a1 = 39;  
% a2 = 39;
% a3 = 35;
% a4 = 9;
% a5 = 9;

a1 = 50000;  
a2 = 50000;
a3 = 50000;
a4 = 10000;
a5 = 10000;

Q  = [0.001   ,0 ;
      0   ,0.001 ];
  

E = q'*Q*q + sum(a1*(z_mid_avgs - x_q).^2 + a2*(y_mid_avgs-z_q).^2) + a3*(arclen_mean-arclen).^2+ a4*(z_ds-x_fkin(end)).^2 + a5*(y_ds-z_fkin(end)).^2;


