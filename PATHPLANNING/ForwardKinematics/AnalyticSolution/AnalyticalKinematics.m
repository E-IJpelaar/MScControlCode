clear all;close all;clc;
%% Parameters
L = 1;

K1 = 1;  % curvatures
K2 = 0;
K3 = 0;
E1 = 0;  % strains
E2 = 0;
E3 = 1;
K = [K1;K2;K3];
E = [E1;E2;E3];
xi_ac = [K;E];   % xi with a(ctive) and c(onctrained) strains/curvatures

sigma = linspace(0,L,100);
g = cell(1,length(sigma));
% e_hat = cell(1,length(sigma));

g0 = [1,0,0,0;
      0,1,0,0;
      0,0,1,0;
      0,0,0,1];
g{1,1} = g0;
I4 = eye(4,4);

for ii = 2:length(sigma)

  e_hat = [0,0,1,0;
           0,0,0,0;
           -1,0,0,1;
           0,0,0,0];     
  k = e_hat(1:3,4);
  theta = k;
  theta2 = k.'*k;
  theta3 = k.'*k*k
%     theta2 = k.'*k;
%     theta3 = k.'*k*k.';
%     
    
    term1 = I4;
    term2 = sigma(ii)*e_hat;
    term3 = (1/theta2)*(1-cos(sigma(ii)*theta))*e_hat^2;
    term4 = (1./theta3)*(sigma(ii)*theta - sin(sigma(ii)*theta))*e_hat^3;
%     
    
    g{1,ii} = g{1}*(term1 + term2);% + term3 + term4);
    y(ii) = g{1,ii}(2,4);
    z(ii) = g{1,ii}(3,4);
    
end


figure(1)
plot(y,z,'LineWidth',1.5)
hold on;grid on;