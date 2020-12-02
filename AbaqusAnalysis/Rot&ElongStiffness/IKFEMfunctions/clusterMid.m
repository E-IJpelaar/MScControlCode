function [y_mean, z_mean] = clusterMid(y_mid,z_mid)

n_clusters = 10;     % amount of clusters
T = clusterdata(y_mid,n_clusters);
y_mean = zeros(1,n_clusters);
z_mean = zeros(1,n_clusters);

for ii = 1:n_clusters
    indices_ii = find(T == ii);
    y_mean(ii) = mean(y_mid(indices_ii));
    z_mean(ii) = mean(z_mid(indices_ii));
end

A = [y_mean(:),z_mean(:)];
A = sortrows(A);

y_mean = A(:,1);
z_mean = A(:,2);


