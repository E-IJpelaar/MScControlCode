clear all;close all;clc;tic

changeExtention();  % copy and change ".rpt" to ".txt"

% Bottom Nodes
filename_nodes = 'BottomNodesRot60kMesh.txt'; 
[bot_nodes] = NodesOfInterest(filename_nodes); 

% Top Nodes
filename_nodes = 'TopNodesRot60kMesh.txt'; 
[top_nodes] = NodesOfInterest(filename_nodes); 

% Middle Nodes
filename_nodes = 'MiddleNodes.txt'; 
[mid_nodes] = NodesOfInterest(filename_nodes); 

% Get coordinates for all nodes of interest at a certain pressure
filename_allnodes = 'rot60kPa.txt' ;                
[all_nodes] = allNodes(filename_allnodes);

% Get deformation [x,y,z]
[bot_or,bot_def]= getCoords(bot_nodes,all_nodes);
[top_or,top_def]= getCoords(top_nodes,all_nodes);
[mid_or,mid_def]= getCoords(mid_nodes,all_nodes);

x_u = bot_or(:,1);
y_u = bot_or(:,2);
z_u = bot_or(:,3);

x_ut = top_or(:,1);
y_ut = top_or(:,2);
z_ut = top_or(:,3);

x_d = bot_def(:,1);
y_d = bot_def(:,2);
z_d = bot_def(:,3);

x_dt = top_def(:,1);
y_dt = top_def(:,2);
z_dt = top_def(:,3);

x = mid_def(:,1);
y = mid_def(:,2);
z = mid_def(:,3);

x_ = mid_def(:,1);
y = mid_def(:,2);
z = mid_def(:,3);

figure(2)
plot3(x_u,-z_u,y_u,'bo')
hold on;grid on;
plot3(x_dt,-z_dt,y_dt,'ro')
plot3(x_ut,-z_ut,y_ut,'bo')
plot3(x_d,-z_d,y_d,'ro')
plot3(x,-z,y,'ro')
xlabel('x [mm]');ylabel('y [mm]');zlabel('z [mm]');
view(90,0);% title('Deformation for single bellow load of 50kPa')
legend('undeformed','deformed')
axis([-40 60 -60 60 0 120  ])
% 
% 
% figure(2)
% plot3(all_nodes(:,5),all_nodes(:,7),all_nodes(:,6),'o')



% [n,V,p] = affine_fit(bottom_original)

% %% Calculate mean displacement for all these nodes, in x,y,z
% d_x = mean(coords(:,5) - coords(:,2));
% d_y = mean(coords(:,6) - coords(:,3));
% d_z = mean(coords(:,7) - coords(:,4));
%  
% def = [d_x,d_y,d_z]
% 
% coord_original = coords(:,2:4);
% coord_deformed = coords(:,5:7);
% 
% figure(1)
% plot3(coord_original(:,1),coord_original(:,2),coord_original(:,3),'bo')
% hold on; 
% plot3(coord_deformed(:,1),coord_deformed(:,2),coord_deformed(:,3),'ro')
% xlabel('x coord');zlabel('y coord');zlabel('z coord')
% % [n,V,p] = affine_fit(X)

save('60kPabending.mat','z','y')

