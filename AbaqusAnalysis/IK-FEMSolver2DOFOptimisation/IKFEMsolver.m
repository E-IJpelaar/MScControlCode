clear all;close all; clc; tic;
% FEM simulations are in the z-y plane
% The IK script is in the x-z plane
% This is very inconvenient, so when adapting the scripts, espescially
% while plotting, take this into account and check if you plot is correct.

%% IK settings
mm2m = 1e-3;
L = 64.5*mm2m;           % [m] length of the actuator
Nmode = 1;
shape = 'cheby';           % type of shape function to be used
epsilon = 0.001;            % max error norm for IK solution  

% data_location = 'C:\Users\s159261\OneDrive - TU Eindhoven\Documents\GitHub\MScControlCode\AbaqusAnalysis\Rot&ElongStiffness\ForceData';
% data_location = 'C:\Users\s159261\OneDrive - TU Eindhoven\Documents\GitHub\MScControlCode\AbaqusAnalysis\Rot&ElongStiffness\ElongData'; % select folder in which the data is stored
data_location = 'C:\Users\s159261\OneDrive - TU Eindhoven\Documents\GitHub\MScControlCode\AbaqusAnalysis\DataFiles\RotationAnalysis';
%% Get nodal information from FEM simulations
         
current_file = 'rot50kPa.txt';

% all nodes
filename_allnodes = current_file ;                
[all_nodes] = allNodes(filename_allnodes);

% nodes of interest
% bottom nodes
filename_nodes = 'BottomNodes60kMesh.txt'; 
[bot_nodes] = NodesOfInterest(filename_nodes);
% centre line nodes
filename_nodes = 'MiddleNodes60kMesh.txt'; 
[mid_nodes] = NodesOfInterest(filename_nodes); 
% top nodes (for orientation and end-effector location)
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


[y_mid_avg, z_mid_avg] = clusterMid(y_mid,z_mid); % get the mean values of the data clusters of the deformed mid


% Fit planes through deformed plane and original plane
[n_bot,~,p_bot] = affine_fit(bot_def(:,2:3));
[n_top,~,p_top] = affine_fit(top_def(:,2:3));

% end-effector cooridinates
theta_d = acos(dot(n_bot,n_top));     % orientation of the deformed plane
y_d = p_top(1);                       % point on plane
z_d = p_top(2);                       % point on plane


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
axis equal

% mm to m
z_ds = z_d*mm2m;
y_ds = y_d*mm2m;
y_mid_avgs = y_mid_avg*mm2m;
z_mid_avgs = z_mid_avg*mm2m;


% intermediate plot 
figure(2)
plot(z_ds,y_ds,'kx','MarkerSize',15,'LineWidth',1.5)
hold on; grid on;
plot(z_mid*mm2m,y_mid*mm2m,'bo')
plot(z_mid_avgs,y_mid_avgs,'rx','MarkerSize',15,'LineWidth',1.5)
xlabel('z [mm]');ylabel('y [mm]')
legend('end-effector position','middle nodes FEM','cluster mean value')
axis equal

%% Optimization
x_d = [z_ds;y_ds];                                    % scaled end-effector position
[x0,q0] = InverseKinematics2DOF(x_d,epsilon,Nmode,L); % initial q0 solving IK
% q0 = rand(2*Nmode,1);
fun = @(x)errorFunction(x,z_ds,y_ds,z_mid_avgs,y_mid_avgs,Nmode,shape,L); % write as a function where only x is optimized
% lb = -1*ones(1,2*Nmode);
% ub = ones(1,2*Nmode);
[q_opt,~,exitflag] = fmincon(fun,q0);%,[],[],[],[],lb,ub);                 % optimization

% Optimal solution
[x_opt_f,z_opt_f] = funcKinematics(Nmode,shape,q_opt,L);
E = errorFunction(q_opt,z_ds,y_ds,z_mid_avgs,y_mid_avgs,Nmode,shape,L);
toc


% 
figure(3)
plot(z_mid_avg*mm2m,y_mid_avg*mm2m,'x','MarkerSize',15,'LineWidth',1.5)
hold on; grid on; 
plot(z_d*mm2m,y_d*mm2m,'kx','MarkerSize',15,'LineWidth',1.5)
plot(x_opt_f,z_opt_f,'LineWidth',1)
plot(x0(:,1),x0(:,2),'LineWidth',1)
legend('mean of the mid nodes','end-effector','optimized IK solution','IK solution (q0)')
xlabel('z [mm]');ylabel('y [mm]')
axis equal
