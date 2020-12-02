clear all;close all;clc

L_act = 64.5;                  % [mm] actuator length
Nmode = 1;                     % Nmode used in IKFEMsolver2DOF script        


load topforce_20201118T192437.mat
data = sortrows(data.',1).';   % sort data by increasing Force


F = data(1,:);
q1= data(2:2+(Nmode-1),:);
q2= data(2+Nmode:1+2*Nmode,:);


figure(1)
plot(F,q1,'bo');
hold on;grid on;
plot(F,q2,'rx');
legend('\kappa','\epsilon')
xlabel('q [-]');ylabel('Force [N]')

ke = q1*L_act;
ee = q2*L_act;

% P = polyfit(ee,F,1);
% F_fit = polyval(P,)
% 

% pressure as function of absolute elongation and curvature
figure(4)
hold on;grid on;
plot(F,ee,'ro');
legend('elongation')
xlabel('Force [N]');ylabel('Elongation [mm]')
axis([0 30 0 40])

