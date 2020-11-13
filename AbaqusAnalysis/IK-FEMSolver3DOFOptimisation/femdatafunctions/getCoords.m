function [original_coords,deformed_coords] = getCoords(active_nodes,all_nodes)

% [#node u_x u_y u_z d_x d_y d_z]

coords = zeros(length(active_nodes),7);
for ii = 1:length(active_nodes)
    node_ii = active_nodes(ii); 
    coords(ii,:) = all_nodes(node_ii,:);
end

original_coords = coords(:,2:4);
deformed_coords = coords(:,5:7);
