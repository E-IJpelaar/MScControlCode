clear all;close all;clc;

sample_delay = 3;

T = [];
P = [];
U = [];
V = [];

for ii =1:6
   
%     for jj = 1:3
jj = 1;

        file_name = [num2str(ii),'V',num2str(jj),'.txt'];
        data = load(file_name);
        
        t = data(1:end-sample_delay,1);
        p = data(sample_delay+1:end,3)*0.1;
        u = data(sample_delay+1:end,5);
        v = u/341.33333;
        
        T = [T;t];
        P = [P;p];
        U = [U;u];
        V = [V;v];

%         if ii == 1
%             b = 'k';
%         elseif ii == 2;
%             b = 'c';
%         elseif ii == 3;
%             b = 'm';
%         elseif ii == 4;
%             b = 'g';
%         elseif ii == 5;
%             b = 'r';
%         elseif ii == 6;
%             b = 'b'
%         end
        
         

        figure(1)
        plot(t,p,'LineWidth',1.5)
        hold on;grid on;
        
%         figure(2)
%         plot(t,p)
%         hold on;grid on;
        
    
    end
% end

figure(1)
xlabel('Time [s]');ylabel('Pressure [kPa]')
axis([0 2 -10 80])
legend('1V','2V','3V','4V','5V','6V','Orientation','horizontal')

x0 = rand(2,1);
bestxu = fminsearch(@(x) errorPressure(x,T,P,U),x0);
bestxV = fminsearch(@(x) errorPressure(x,T,P,V),x0);

u1 = 341.33;
u2 = 682.67;
u3 = 1024;
u4 = 1365.33;
u5 = 1706.67;
u6 = 2048;

% figure(1)
% % plot(t,(u2*bestxu(1)*(1-exp(-t/bestxu(2)))))
% % plot(t,(u3*bestxu(1)*(1-exp(-t/bestxu(2)))))
% plot(t,(u4*bestxu(1)*(1-exp(-t/bestxu(2)))))
% plot(t,(u5*bestxu(1)*(1-exp(-t/bestxu(2)))))
% plot(t,(u6*bestxu(1)*(1-exp(-t/bestxu(2)))))
% xlabel('Time [s]');ylabel('Pressure Difference [kPa]')
% xlim([0 2]);ylim([0 80])
% disp(['Pressure = pwm',num2str(bestxu(1)),'[1-exp(-t/',num2str(bestxu(2)),')]'] )

% V1 = 1;
% V2 = 2;
V3 = 3;
V4 = 4;
V5 = 5;
V6 = 6;

% figure(2)
% % plot(t,(V2*bestxV(1)*(1-exp(-t/bestxV(2)))))
% % plot(t,(V3*bestxV(1)*(1-exp(-t/bestxV(2)))))
% plot(t,(V4*bestxV(1)*(1-exp(-t/bestxV(2)))))
% plot(t,(V5*bestxV(1)*(1-exp(-t/bestxV(2)))))
% plot(t,(V6*bestxV(1)*(1-exp(-t/bestxV(2)))))
% xlabel('Time [s]');ylabel('Pressure Difference [kPa]')
% xlim([0 2]);ylim([0 80])
% % legend('Experimental Data','4V fit','5V fit','6V fit')

disp(['Pressure = V',num2str(bestxV(1)),'[1-exp(-t/',num2str(bestxV(2)),')]'] )