clear all;close all; clc;
% FEM simulations are in the z-y plane
% The IK script is in the x-z plane
% This is very inconvenient, so when adapting the scripts, espescially
% while plotting, take this into account and check if you plot is correct.

mm2m = 1e-3; % converting factor
%% Get nodal information from FEM simulations
% all nodes
filename_allnodes = 'rot60kPa.txt' ;                
[all_nodes] = allNodes(filename_allnodes);

% nodes of Interest
% bottom nodes
filename_nodes = 'BottomNodes60kMesh.txt'; 
[bot_nodes] = NodesOfInterest(filename_nodes);
% centre line nodes
filename_nodes = 'MiddleNodes60kMesh.txt'; 
[mid_nodes] = NodesOfInterest(filename_nodes); 
% top nodes (for orientation)
filename_nodes = 'TopNodes60kMeshtest.txt'; 
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

% Fit planes through deformed plane and original plane
[n_bot,~,p_bot] = affine_fit(bot_def(:,2:3));
[n_top,~,p_top] = affine_fit(top_def(:,2:3));

%% FEM data with plane fit

figure(1)
plot(z_top,y_top,'ro') 
hold on;grid on; box on;
plot(z_mid,y_mid,'ro')
plot(z_bot,y_bot,'bo')
plot(p_top(2),p_top(1),'rx','MarkerSize',15,'MarkerFaceColor','r')
hold on;grid on;
plot(p_bot(2),p_bot(1),'bx','MarkerSize',15,'MarkerFaceColor','b')
quiver(p_top(2),p_top(1),n_top(2)/2,n_top(1)/2,50,'r','linewidth',2)
quiver(p_bot(2),p_bot(1),n_bot(2)/2,n_bot(1)/2,50,'b','linewidth',2)
xlabel('z [mm]');ylabel('y [mm]')
legend('deformed top plate','deformed centre line','undeformed bottom plate','point on plane top','point on plane bottom','normal vector plane','normal vector plane')
axis equal

%% Find IK model

theta_d = acos(dot(n_bot,n_top));     % orientation of the deformed plane
y_d = p_top(1);                       % point on plane
z_d = p_top(2);                         

L_act = 64.5e-3; % m
x_d = [-theta_d;z_d*mm2m;y_d*mm2m];

Nmode = 1;                 % # shape functions to approximate strain/curvature
epsilon = 0.0001;          % max error norm

[theta_opt, x_opt,q_opt] = InverseKinematics3DOF(x_d,epsilon,Nmode,L_act);


figure(2)
plot(z_d*mm2m,y_d*mm2m,'bx','MarkerSize',15)
hold on;grid on;
plot(-x_opt(:,2),x_opt(:,1),'b','LineWidth',2)
plot(z_mid*mm2m,y_mid*mm2m,'ro')
xlabel('z [mm]');ylabel('y [mm]')
legend('End effector FEM simulation','Inverse Kinematic Solution','Deformed centreline FEM')
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







