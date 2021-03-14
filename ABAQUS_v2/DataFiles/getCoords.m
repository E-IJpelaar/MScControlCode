function [original_coords,deformed_coords] = getCoords(active_nodes,all_nodes)

% This function extracts the original and deformed coordinated of the
% active nodes/nodes of interest. To this end, an array with the active
% nodes, and a matrix with ALL node numbers including their displacement is
% needed.

% [#node u_x u_y u_z d_x d_y d_z](u = undeformed, d = deformed)

coords = zeros(length(active_nodes),7);
for ii = 1:length(active_nodes)
    node_ii = active_nodes(ii); 
    coords(ii,:) = all_nodes(node_ii,:);
end

original_coords = coords(:,2:4);
deformed_coords = coords(:,5:7);
