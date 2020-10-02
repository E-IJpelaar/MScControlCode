clear all;close all;clc;

l = 1;  % [m]
g0 = [1;0;0;0;
      0;1;0;0;
      0;0;1;0;
      0;0;0;1];

[l,g] = ode45(@(l,g) ode_g(l,g),[0,l],g0);


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