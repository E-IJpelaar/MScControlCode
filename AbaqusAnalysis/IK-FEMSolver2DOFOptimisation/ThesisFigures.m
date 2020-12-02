clear all;close all;clc
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
[mid_or,mid_def]= getCoords(mid_nodes,all_nodes);
[top_or,top_def]= getCoords(top_nodes,all_nodes);

% Rearrange [x,y,z]
%% undeformed
% x_bot = bot_or(:,1);
y_bot = bot_or(:,2);
z_bot = bot_or(:,3);

x_mid = mid_or(:,1);
y_mid = mid_or(:,2);
z_mid = mid_or(:,3);

x_top = top_or(:,1);
y_top = top_or(:,2);
z_top = top_or(:,3);

%% deformed
% x_bot = bot_def(:,1);
% y_bot = bot_def(:,2);
% z_bot = bot_def(:,3);
% 
% x_mid = mid_def(:,1);
% y_mid = mid_def(:,2);
% z_mid = mid_def(:,3);
% 
% x_top = top_def(:,1);
% y_top = top_def(:,2);
% z_top = top_def(:,3);

[y_mid_avg, z_mid_avg] = clusterMid(y_mid,z_mid); % get the mean values of the data clusters of the deformed mid


% Fit planes through deformed plane and original plane
[n_bot,~,p_bot] = affine_fit(bot_or(:,2:3));
[n_top,~,p_top] = affine_fit(top_or(:,2:3));

% end-effector cooridinates
theta_d = acos(dot(n_bot,n_top));     % orientation of the deformed plane
y_d = p_top(1);                       % point on plane
z_d = p_top(2);                       % point on plane


%% FEM data with plane fit

figure(1)
plot(z_top,y_top,'bo') 
hold on;grid on; box on;
plot(z_mid,y_mid,'bo')
plot(z_bot,y_bot,'bo')
% plot(p_top(2),p_top(1),'kx','MarkerSize',15,'MarkerFaceColor','r','LineWidth',2)
hold on;grid on;
% plot(p_bot(2),p_bot(1),'bx','MarkerSize',15,'MarkerFaceColor','b','LineWidth',2)
quiver(p_top(2),p_top(1),n_top(2)/2,n_top(1)/2,30,'k','linewidth',2)
quiver(p_bot(2),p_bot(1),n_bot(2)/2,n_bot(1)/2,30,'k','linewidth',2)
xlabel('x [mm]');ylabel('y [mm]')
% legend('underformed' )
axis equal
axis([-60 60 -10 100])



% figure(1)
% plot(-z_top,y_top,'ro') 
% hold on;grid on; box on;
% plot(-z_mid,y_mid,'ro')
% plot(-z_bot,y_bot,'ro')
% % plot(p_top(2),p_top(1),'kx','MarkerSize',15,'MarkerFaceColor','r','LineWidth',2)
% hold on;grid on;
% % plot(p_bot(2),p_bot(1),'bx','MarkerSize',15,'MarkerFaceColor','b','LineWidth',2)
% quiver(-p_top(2),p_top(1),-n_top(2)/2,n_top(1)/2,30,'k','linewidth',2)
% quiver(-p_bot(2),p_bot(1),-n_bot(2)/2,n_bot(1)/2,30,'k','linewidth',2)
% xlabel('x [mm]');ylabel('y [mm]')
% % legend('underformed' )
% axis equal
% axis([-60 60 -10 100])
