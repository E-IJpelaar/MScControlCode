clear all;close all;clc;

L_act = 64.5;                  % [mm] actuator length
Nmode = 1;                     % Nmode used in IKFEMsolver2DOF script        


load topforce_20201118T201444.mat
dataf = sortrows(data.',1).';   % sort data by increasing Force

dataf = [zeros(4,1),dataf];

dataf(:,end) = [];
F = dataf(1,:);
for kk = 1:2:7
F(kk) = F(kk)+0.5;
end

q1f= dataf(2:2+(Nmode-1),:);
q2f= dataf(2+Nmode:1+2*Nmode,:);

clearvars data
load elongationdataNEW.mat
datap = sortrows(data.',1).';   % sort data by increasing pressure
datap = [zeros(4,1),datap];
datap(:,end) = [];
% % assign data
p1 = datap(1,:);
p2 = datap(1,:);
q1p= datap(2:2+(Nmode-1),:);
q2p= datap(2+Nmode:1+2*Nmode,:);


figure(5)
plot(p1,q2p*L_act,'o')
hold on; grid on;
plot(F,q2p*L_act,'x')
ylabel('Elongation'); xlabel('Red Force [N],Blue Pressure [kPa]')
legend('Pressure','Force')

fun = @(x)mappingForce(x,p1,F);
x0 = 3;
[alpha,~,exitflag]= fmincon(fun,x0,[],[],[],[],[],[])


figure(6)
plot(alpha*p1,q2p*L_act,'o')
hold on; grid on;
plot(F,q2f*L_act,'x')
ylabel('Elongation [mm]');xlabel('Force [N]')
legend('mapped Pressure','Force')






% figure(1)
% % yyaxis left
% plot(q1f,F,'bo');
% hold on;grid on;
% plot(q2f,F,'bx');
% ylabel('Force [N]')
% % yyaxis right
% plot(q1p,p1,'ro')
% plot(q2p,p1,'rx')
% % legend('\kappa','\epsilon')
% xlabel('q [-]');ylabel('Pressure [kPa]')
% 
% kf = q1f*L_act;
% ef = q2f*L_act;

% P = polyfit(ee,F,1);
% F_fit = polyval(P,)
% 

% pressure as function of absolute elongation and curvature
% figure(4)
% hold on;grid on;
% plot(F,ee,'ro');
% legend('elongation')
% xlabel('Force [N]');ylabel('Elongation [mm]')
% axis([0 30 0 40])
