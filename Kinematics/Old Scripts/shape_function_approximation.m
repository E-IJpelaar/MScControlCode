clear all;close all;clc;

l = 1;

% k1 = 0.4; 
% k2 = 0.6;
% k3 = 0.2;
% 
% e1 = 0.5;
% e2 = 0.5;
% e3 = 0.2;
% 
% K = [k1;k2;k3];
% E = [e1;e2;e3];
% 
% 
% xi = [K;E];
% Ba = eye(6);

g0 = [1;0;0;0;
      0;1;0;0;
      0;0;1;0;
      0;0;0;1];


[l,g] = ode45(@(l,g) odefnc(l,g) , [0 l],g0)

for ii = 1:length(l)
G{ii} = reshape(g(ii,:),[4,4]);
xdata(ii) = G{ii}(1,4);
ydata(ii) = G{ii}(2,4);
zdata(ii) = G{ii}(3,4);
end

[arclen,seglen] = arclength(xdata,ydata,zdata,'l');

figure(1)

plot3(xdata,ydata,zdata)
xlabel('x'); ylabel('y');zlabel('z')
grid on; box on;

