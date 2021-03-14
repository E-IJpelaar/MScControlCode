clear all;close all;clc;
%% load files
load mapping.mat             % load file as created with InputMapping.m
p1 = datap(1,:);             % first row is sorted pressure
p2 = p1;                     % elongation, p2 = p1
pq1 = datap(2,:);
pq2 = datap(3,:);            % elongation

F_mapped = H(1,:)*[p1;p2];   % force by mapping
F_mapped = [sort(-F_mapped),F_mapped];  % assume: -F => -elongatoin
pq2 = [sort(-pq2),pq2];                 % assume: -F => -elongatoin

% fit stiffness model
options = optimoptions('fmincon','Display','iter','MaxFunctionEvaluations',1e4 );
fun = @(x)stiffnessModel(x,pq2,F_mapped);
x0 = 1.0e+03 *[     1.3937    1.3776    0.0003];
lb = zeros(3,1);
[alpha,fval,exitflag] = fmincon(fun,x0,[],[],[],[],[lb],[],@nonlcon,options);
fval
% evaluate for a range elongation
q2_range = linspace(-0.6,0.6,100);
[F_fit,~]= evalForce(alpha,q2_range);


figure(1)
plot(F_mapped((length(pq2)/2)+1:end),pq2((length(pq2)/2)+1:end),'bo','MarkerSize',8,'LineWidth',1.5) % only half is simulated, the rest is assumed
hold on;grid on; box on;
plot(F_fit,q2_range,'r','LineWidth',1.5)
xlabel('Force [N]','FontSize',12); ylabel('Elongation \epsilon [-]','FontSize',12)
legend('FEA with input mapping','Hyper-elastic stiffness model fit','FontSize',12,'Location','southeast')



figure(2)
plot(p1,-pq1,'bx','MarkerSize',8,'LineWidth',1.5) % only half is simulated, the rest is assumed
hold on;grid on; box on;
plot(p1,pq2((length(pq2)/2)+1:end),'ro','MarkerSize',8,'LineWidth',1.5)
xlabel('Pressure [kPa]','FontSize',12); ylabel('Output','FontSize',12)
legend('Curvature \kappa [1/m]','Elongation \epsilon [-]','FontSize',12,'Location','southeast')


figure(3)
% yyaxis left
% plot(p1,-pq1,'bx','MarkerSize',8,'LineWidth',1.5) % only half is simulated, the rest is assumed
hold on;grid on; box on;
% ylabel('Curvature \kappa [1/m]')

% yyaxis right
plot(p1,pq2((length(pq2)/2)+1:end),'ro','MarkerSize',8,'LineWidth',1.5)
ylabel('Elongation \epsilon [-]')
xlabel('Pressure [kPa]','FontSize',12); ylabel('Elongation [-]','FontSize',12)
% legend('Curvature \kappa [1/m]','Elongation \epsilon [-]','FontSize',12,'Location','southeast')