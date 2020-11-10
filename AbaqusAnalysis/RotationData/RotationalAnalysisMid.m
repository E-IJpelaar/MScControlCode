clear all;close all;clc;tic

changeExtention();  % copy and change ".rpt" to ".txt"
% filename_nodes = 'BottomNodesRot60kMesh.txt'; 
% [bottom_nodes] = NodesOfInterest(filename_nodes); 
% 
% filename_nodes = 'TopNodesRot60kMesh.txt'; 
% [top_nodes] = NodesOfInterest(filename_nodes); 
% 
% 
% filename_allnodes = 'rot50kPa.txt' ;                
% [all_nodes] = AllNodes(filename_allnodes);
% [bottom_original,bottom_deformed]= getCoords(bottom_nodes,all_nodes);
% 
% 
% [top_original,top_deformed]= getCoords(top_nodes,all_nodes);

%% Middle Nodes
filename_nodes = 'MiddleNodes.txt'; 
[mid_nodes] = NodesOfInterest(filename_nodes); 
filename_allnodes = 'rot50kPa.txt' ;                
[all_nodes] = AllNodes(filename_allnodes);
[mid_original,mid_deformed]= getCoords(mid_nodes,all_nodes);

% figure(1)
% plot3(mid_original(:,1),mid_original(:,3),mid_original(:,2),'bx')
% hold on
% plot3(mid_deformed(:,1),mid_deformed(:,3),mid_deformed(:,2),'rx')
% % view(90,0)



figure(2)
plot(mid_deformed(:,3),mid_deformed(:,2),'rx')
hold on;







% x_u = bottom_original(:,1);
% y_u = bottom_original(:,2);
% z_u = bottom_original(:,3);

% x_ut = top_original(:,1);
% y_ut = top_original(:,2);
% z_ut = top_original(:,3);

% x_d = bottom_deformed(:,1);
% y_d = bottom_deformed(:,2);
% z_d = bottom_deformed(:,3);

% x_dt = top_deformed(:,1);
% y_dt = top_deformed(:,2);
% z_dt = top_deformed(:,3);

% figure(1)
% plot3(x_u,z_u,y_u,'bx')
% hold on
% plot3(x_ut,z_ut,y_ut,'bo')
% plot3(x_dt,z_dt,y_dt,'ro')
% plot3(x_d,z_d,y_d,'ro')
% view(90,0)
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

