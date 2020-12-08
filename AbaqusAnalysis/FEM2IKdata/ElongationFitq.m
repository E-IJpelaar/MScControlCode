clear all;close all;clc;
%% Variables
L_act = 64.5;                  % [mm] actuator length
Nmode = 1;                     % Nmode used in IKFEMsolver2DOF script
% A_eff = 0.0006;                % current force mapping F = p*A_eff
H =   [0.0003,0.0003;                  % Future force mapping
      0.0003,-0.0003];                    

%% load and sort rotation data 
load elongationdataNEW.mat
data = sortrows(data.',1).';   % sort data by increasing pressure

% % assign data
p1 = data(1,:);
p2 = data(1,:);
q1e= data(2:2+(Nmode-1),:);
q2e= data(2+Nmode:1+2*Nmode,:);

p = [p1;p2];


% pressure as function of q
figure(1)
plot(p1,q1e,'bo');
hold on;grid on;
plot(p1,q2e,'rx');
legend('\kappa','\epsilon')
xlabel('Pressure kPa');ylabel('q [-]')

ke = q1e*L_act;
ee = q2e*L_act;

% pressure as function of absolute elongation and curvature
figure(2)
plot(p1,ee,'ro');
hold on;grid on;
legend('elongation')
xlabel('Pressure kPa');ylabel('Elongation [mm]')

% q as function of Force
Fe = H(1,:)*p*1e3;                  % current force mapping
Fe = [0,Fe];                        % manually add point 0
q2e = [0,q2e];
% 

options = optimoptions('fmincon','Display','iter','MaxFunctionEvaluations',1e4 );
fun = @(x)stiffnessModel(x,q2e,Fe);
x0 = [2500 2500 1];
[alpha,~,exitflag] = fmincon(fun,x0,[],[],[],[],[0;0;0],[],@nonlcon,options)



qse = linspace(-0.6,0.6,100);
Ke = alpha(1) + alpha(2).*((tanh(alpha(3).*qse)).^2 -1);


figure(3)
plot(q2e,Fe,'ro');
hold on;grid on;
plot(qse,Ke.*qse,'r')
ylabel(' Force [N]')
legend('Simulated','Hyper-Elastic Model Fit')
xlabel('q_1 [-]');




% %% Rotational part
% 
% load rotdata_20201118T143936.mat
% data = sortrows(data.',1).';   % sort data by increasing pressure
% 
% %% Assumption curvatures in + and - direction are equal. We only regard the + curvature
% p1 = data(1,:);
% p2 = zeros(1,length(data));
% q1k= -data(2:2+(Nmode-1),:);
% q2k= data(2+Nmode:1+2*Nmode,:);
% 
% p = [p1;p2];
% 
% 
% % pressure as function of q
% figure(1)
% plot(p1,q1k,'bo');
% hold on;grid on;
% plot(p1,q2k,'rx');
% legend('\kappa','\epsilon')
% xlabel('Pressure kPa');ylabel('q [-]')
% 
% kk = q1k*L_act;
% ek = q2k*L_act;
% 
% % pressure as function of absolute elongation and curvature
% figure(2)
% plot(p1,kk,'bo');
% hold on;grid on;
% plot(p1,ek,'ro');
% legend('k','e')
% legend('curvature','elongation')
% xlabel('Pressure kPa');ylabel('Deviation [mm]')
% 
% % q as function of Force
% Fk = H(2,:)*p*1e3;                  % current force mapping
% Fk = [0,Fk];                        % manually add point 0
% q2k = [0,q2k];
% q1k = [0,q1k];
% 
% options = optimoptions('fmincon','Display','iter','MaxFunctionEvaluations',1e4 );
% fun = @(x)stiffnessModel(x,q1k,Fk);
% x0 = [10000 10000 0.1];
% [alpha,~,exitflag] = fmincon(fun,x0,[],[],[],[],[0;0;0],[],@nonlcon,options)
% 
% 
% 
% qsk = linspace(-1.2,1.2,100);
% Kk = alpha(1) + alpha(2).*((tanh(alpha(3).*qsk)).^2 -1);
% 
% 
% figure(3)
% plot(q1k,Fk,'ro');
% hold on;grid on;
% plot(qsk,Kk.*qsk,'r')
% ylabel(' Force [N]')
% legend('Simulated','Hyper-Elastic Model Fit')
% xlabel('q_1 [-]');


