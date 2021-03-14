clear all;close all;clc;

addpath('C:\Users\s159261\OneDrive - TU Eindhoven\Documents\GitHub\MScControlCode\StepResponse\DataStep1802')

for ii = 1:6
    for jj = 1:3
        
       data = load([num2str(ii),'V',num2str(jj),'.txt']);
       sample_loss = 2;
       t = data(1:end-sample_loss,1);
       p = data(sample_loss+1:end,3)*0.100;
       u = data(1,5);
       
       figure(1)
       plot(t,p)
       hold on;grid on;
       xlabel('Time [s]');ylabel('Relative Pressure [kPa]')
       xlim([0 2]);ylim([-5 80])
      
       %plot(t,725*(1-exp(-t./0.09)))
       
%       x = fmincon(@(x) errorPressure(x,t,p,u),[0,0]);
      
%       A(ii,jj) = x(1);
%       B(ii,jj) = x(2);
       
     
    end
    %legend('6V','5V','4V','3V','2V','1V')
end


% function E = errorPressure(x,t,p,u)
% 
% K = x(1);
% tau = x(2);
% 
% for ii = 1:length(t)
%     
%     y = K*(1+exp(-t(ii)/tau));
%     E = p - y*u;
% 
% end
% 
% 
% end




