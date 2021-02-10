clear all;close all;clc;

% a= dir( '*.txt');
% n_files =size(a,1);
% 
% % 30 33 34 .35 36 38 
% % 31 32 37 40
for ii = [36]
%    
    data = load(strcat(['data',num2str(ii),'.txt']));
    data = data(1:250,:);
%    
%     figure(1)
%     hold on;grid on;
%     plot(data(:,1),data(:,4)*0.1,'DisplayName',['K_p= ',num2str(data(1,6)),' K_d= ',num2str(data(1,8))],'LineWidth',1.5);
%     xlabel('Time [s]');ylabel('Pressure difference [kPa]');
%     
% 
%     figure(2)
%     hold on;grid on;
%     plot(data(:,1),data(:,5)*(12/4096),'DisplayName',['K_p= ',num2str(data(1,6)),' K_d= ',num2str(data(1,8))],'LineWidth',1.5);
% 
%     xlabel('Time [s]');ylabel('Control Input [V]');
% 
end
% 
% legend show
% 
% figure(1)
% % subplot(1,2,1)
% plot(data(:,1),data(:,2)*0.1,'LineWidth',1)
% ylim([0 12])
% % axis equal
% 
% 
% % figure(2)
% % % subplot(1,2,2)
% % yline(12,'LineWidth',1)
% % ylim([0 14])
% % % axis equal
% 
% 
[p,S] = polyfit(data(:,1),data(:,4)*0.1,3);
x = linspace(0,2.5,100);
[y_fit,delta] = polyval(p,x,S);
plot(x,y_fit)

t = linspace(0,1,100);
tau =0.3;
z = smoothstep(t);

y = 1-exp(-t/tau);

figure(1)
plot(t,z)
hold on;grid on;
plot(t,y)
legend('brandon', 'erwin')


function y = smoothstep(X)

y = zeros(length(X),1); 

for ii = 1:length(X)
    x = X(ii);
    if x<=0, y(ii,1) = 0;
    elseif (x>0 && x<=1), y(ii,1) = 3*x.^2 -2*x.^3;
    else, y(ii,1) = 1;
    end
end
end

