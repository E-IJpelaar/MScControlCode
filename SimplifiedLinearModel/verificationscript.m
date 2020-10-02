clear all;close all; clc;

l = 1;
g0 = eye(4);
g0 = reshape(g0,[],1);

[l,g] = ode45(@(l,g) ode_g(l,g) ,[0 l],g0); % original ode


for ii = 1:length(l)
G{ii} = reshape(g(ii,:),[4,4]);
xdata(ii) = G{ii}(1,4);
ydata(ii) = G{ii}(2,4);
zdata(ii) = G{ii}(3,4);
end

clear g l 
l = 1;

[l,g] = ode45(@(l,g) odefnc(l,g),[0 l],g0); % shape fnc

for ii = 1:length(l)
G2{ii} = reshape(g(ii,:),[4,4]);
xdata2(ii) = G2{ii}(1,4);
ydata2(ii) = G2{ii}(2,4);
zdata2(ii) = G2{ii}(3,4);
end


figure(1)

plot3(xdata,ydata,zdata)
grid on; box on; hold on;
plot3(xdata2,ydata2,zdata2)
xlabel('x'); ylabel('y');zlabel('z')
legend('original ODE','shape approx')



