clear all;close all; clc; tic;
% FEM simulations are in the z-y plane
% The IK script is in the x-z plane
% This is very inconvenient, so when adapting the scripts, espescially
% while plotting, take this into account and check if you plot is correct.

%% IK settings
L_act = 0.0645;              % [mm] length of the actuator
Nmode = 1;                 % # shape functions to approximate strain/curvature
shape = 'cheby';           % type of shape function to be used
epsilon = 0.05;            % max error norm for IK solution  
mm2m  = 1e-3;
%% Get nodal information from FEM simulations
% all nodes
filename_allnodes = 'rot40kPa.txt' ;                
[all_nodes] = allNodes(filename_allnodes);

% nodes of Interest
% bottom nodes
filename_nodes = 'BottomNodes40kMesh.txt'; 
[bot_nodes] = NodesOfInterest(filename_nodes);
% centre line nodes
filename_nodes = 'MiddleNodes40kMesh.txt'; 
[mid_nodes] = NodesOfInterest(filename_nodes); 
% top nodes (for orientation)
filename_nodes = 'TopNodes40kMeshtest.txt'; 
[top_nodes] = NodesOfInterest(filename_nodes);


% Get deformation [x,y,z]
[bot_or,bot_def]= getCoords(bot_nodes,all_nodes);
[~,mid_def]= getCoords(mid_nodes,all_nodes);
[~,top_def]= getCoords(top_nodes,all_nodes);

% Rearrange [x,y,z]
x_bot = bot_or(:,1);
y_bot = bot_or(:,2);
z_bot = bot_or(:,3);
 
x_top = top_def(:,1);
y_top = top_def(:,2);
z_top = top_def(:,3);

x_mid = mid_def(:,1);
y_mid = mid_def(:,2);
z_mid = mid_def(:,3);

[y_mid_avg, z_mid_avg] = clusterMid(y_mid,z_mid); % get the mean values of the data clusters of the deformed mid


% Fit planes through deformed plane and original plane
[n_bot,~,p_bot] = affine_fit(bot_def(:,2:3));
[n_top,~,p_top] = affine_fit(top_def(:,2:3));

% end-effector cooridinates
theta_d = acos(dot(n_bot,n_top));     % orientation of the deformed plane
y_d = p_top(1);                       % point on plane
z_d = p_top(2);                       % point on plane


%% FEM data with plane fit

figure(1)
plot(z_top,y_top,'ro') 
hold on;grid on; box on;
plot(z_mid,y_mid,'ro')
plot(z_bot,y_bot,'bo')
plot(p_top(2),p_top(1),'kx','MarkerSize',15,'MarkerFaceColor','r','LineWidth',2)
hold on;grid on;
plot(p_bot(2),p_bot(1),'bx','MarkerSize',15,'MarkerFaceColor','b','LineWidth',2)
quiver(p_top(2),p_top(1),n_top(2)/2,n_top(1)/2,50,'r','linewidth',2)
quiver(p_bot(2),p_bot(1),n_bot(2)/2,n_bot(1)/2,50,'b','linewidth',2)
xlabel('z [mm]');ylabel('y [mm]')
legend('deformed top plate','deformed centre line','undeformed bottom plate','point on plane top','point on plane bottom','normal vector plane','normal vector plane')

% scale parameters
z_ds = z_d*mm2m;
y_ds = y_d*mm2m;
y_mid_avgs = y_mid_avg*mm2m;
z_mid_avgs = z_mid_avg*mm2m;

% intermediate plot 
figure(2)
plot(z_ds,y_ds,'kx','MarkerSize',15,'LineWidth',1.5)
hold on; grid on;
plot(z_mid*1e-3,y_mid*1e-3,'bo')
plot(z_mid_avgs,y_mid_avgs,'rx','MarkerSize',15,'LineWidth',1.5)
xlabel('z [m]');ylabel('y [m]')
legend('end-effector position','middle nodes FEM','cluster mean value')

%% Optimization
if mean(z_ds) < 0 && mean(y_ds) > 0 ||  mean(z_ds) > 0 && mean(y_ds) < 0
    theta_d = -theta_d;
end
x_d = [theta_d; z_ds;y_ds];  %fit in possitive side                              % scaled end-effector position
% [theta0,x0,q0] = InverseKinematics3DOF(x_d,epsilon,Nmode,L_act);   % initial q0 solving IK

q0 = [-12.5783;0.32];
fun = @(x)errorFunction3DOF(x,theta_d,z_ds,y_ds,z_mid_avgs,y_mid_avgs,Nmode,shape,L_act); % write as a function where only x is optimized
[q_opt,~,exitflag] = fmincon(fun,q0);                 % optimization

% Optimal solution
[theta_opt_f,x_opt_f,z_opt_f] = funcKinematics3DOF(Nmode,shape,q_opt,L_act);
toc

figure(3)
plot(z_mid_avg*mm2m,y_mid_avg*mm2m,'x','MarkerSize',15,'LineWidth',1.5)
hold on; grid on; 
plot(z_d*mm2m,y_d*mm2m,'kx','MarkerSize',15,'LineWidth',1.5)
plot(-z_opt_f,x_opt_f,'LineWidth',1)
% plot(-x0(:,2),x0(:,1),'LineWidth',1)
legend('mean of the mid nodes','end-effector','optimized IK solution','IK solution (q0)')
xlabel('z [m]');ylabel('y [m]')
axis equal



%% Only USE this for a 3D plot, not necessary
% Bottom plane is assumed to be a plane at 

% [n_bot,V,p_bot] = affine_fit(bot_def);
% [n_top,V,p_top] = affine_fit(top_def);


% figure(3)
% plot3(p_top(1),p_top(2),p_top(3),'bo','MarkerSize',10,'MarkerFaceColor','b')
% hold on;grid on;
% plot3(p_bot(1),p_bot(2),p_bot(3),'ro','MarkerSize',10,'MarkerFaceColor','r')
% quiver3(p_top(1),p_top(2),p_top(3),n_top(1)/3,n_top(2)/3,n_top(3)/3,100,'b','linewidth',2)
% quiver3(p_bot(1),p_bot(2),p_bot(3),n_bot(1)/3,n_bot(2)/3,n_bot(3)/3,100,'r','linewidth',2)
% plot3(x_top,y_top,z_top,'bo') 
% plot3(x_mid,y_mid,z_mid,'ro')
% plot3(x_bot,y_bot,z_bot,'bo')
% xlabel('x [mm]');ylabel('y [mm]');zlabel('z [mm]');







