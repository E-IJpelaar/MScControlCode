function [E]= errorFunction3DOF(x,theta_d,z_ds,y_ds,z_mid_avgs,y_mid_avgs,Nmode,shape,L)

q = x;                                             % set q equal to optimized variable x
[theta_fkin,x_fkin,z_fkin] = funcKinematics3DOF(Nmode,shape,q,L);   % based on q determine [x-z] based on forward kinematics

% below the coordinates from the forward kinematic model are compared with
% the data set that is to be fit. Since the forward model gives a well
% discritized output, while only a limited data set is provided, indices
% need to found of the coordinates that match best. Namely, in the objective 
% function both vectors need to be of equal size.

p = zeros(1,length(z_mid_avgs));                   % pre-allocate
for ii = 1:length(z_mid_avgs)                      
%     [~,p(ii)] = min((x_fkin-z_mid_avgs(ii)).^2 + (z_fkin-y_mid_avgs(ii)).^2); 
     [~,p(ii)] = min((-z_fkin-z_mid_avgs(ii)).^2 + (x_fkin-y_mid_avgs(ii)).^2); 
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

a1 = 2500;  
a2 = 2500;
a3 = 1000;
a4 = 5000;
a5 = 5000;
a6 = 4000;

Q  = eye(Nmode*2);
E = a4*(-z_ds-z_fkin(end)).^2 + a5*(y_ds-x_fkin(end)).^2; % this works, see when there is more time
% E = q'*Q*q + sum(a1*(z_mid_avgs - x_q).^2 + a2*(y_mid_avgs-z_q).^2 + a3*(arclen_mean-arclen).^2+ a4*(z_ds-z_fkin(end)).^2 + a5*(y_ds-x_fkin(end)).^2 + a6*(theta_d-theta_fkin).^2 );
