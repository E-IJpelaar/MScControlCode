clear all;close all;clc;
%% load files
load mapping.mat             % load file as created with InputMapping.m
clear datap                  % elongation data is not needed
load PressureRot_20201208T151436.mat     % rotation data
rot_data = sortrows(data.',1).';         % sort data by increasing pressure
rot_data = [zeros(4,1),rot_data];        % add an additional point 0 N = 0 elongation

% assign data
p1 = rot_data(1,:);                 % one bellow is inflated
p2 = zeros(1,length(p1));           % one bellow is left unpressized
pq1 = -rot_data(2,:);               % curvature, q1 = -q1 assumption
pq2 = rot_data(3,:);                % elongation, not needed

M_mapped = H(2,:)*[p1;p2];              % Moment by mapping
M_mapped = [sort(-M_mapped),M_mapped];  % assume: -M => -curvature
pq1 = [sort(-pq1),pq1];                 % assume: -M => -curvature

% fit stiffness model
options = optimoptions('fmincon','Display','iter','MaxFunctionEvaluations',1e4 );
fun = @(x)stiffnessModel(x,pq1,M_mapped);
x0 = [ 3.1  3.1  0.003];
lb = zeros(3,1);
[beta,~,exitflag] = fmincon(fun,x0,[],[],[],[],lb,[],@nonlcon,options);

% evaluate for a range elongation
q1_range = linspace(-18,18,100);
[M_fit,~]= evalForce(beta,q1_range);


figure(1)
plot(M_mapped((length(pq1)/2)+1:end),pq1((length(pq1)/2)+1:end),'bo','MarkerSize',8,'LineWidth',1.5) % only half is simulated, the rest is assumed
hold on;grid on; box on;
plot(M_fit,q1_range,'r','LineWidth',1.5)
xlabel('Moment [Nm]','FontSize',12); ylabel('Curvature \kappa [1/mm]','FontSize',12)
legend('FEA with input mapping','Hyper-elastic stiffness model fit','FontSize',12,'Location','southeast')
xlim([-0.2 0.2])


figure(2)
plot(M_mapped((length(pq1)/2)+1:end),pq1((length(pq1)/2)+1:end),'bo','MarkerSize',8,'LineWidth',1.5) % only half is simulated, the rest is assumed
hold on;grid on; box on;
plot(M_fit,q1_range,'r','LineWidth',1.5)
xlabel('Moment [Nm]','FontSize',12); ylabel('Curvature \kappa [1/m]','FontSize',12)
legend('FEA with input mapping','Hyper-elastic stiffness model fit','FontSize',12,'Location','southeast')
xlim([-0.2 0.2])

figure(3)
plot(p1,pq1((length(pq1)/2)+1:end),'bx','MarkerSize',8,'LineWidth',1.5) % only half is simulated, the rest is assumed
hold on;grid on; box on;
plot(p1,pq2,'ro','MarkerSize',8,'LineWidth',1.5)
xlabel('Pressure [kPa]','FontSize',12); ylabel('Output','FontSize',12)
legend('Curvature \kappa [1/m]','Elongation \epsilon [-]','FontSize',12,'Location','southeast')



figure(4)
yyaxis left
plot(p1,pq1((length(pq1)/2)+1:end),'bx','MarkerSize',8,'LineWidth',1.5) % only half is simulated, the rest is assumed
hold on;grid on; box on;
ylabel('Curvature \kappa [1/m]')

yyaxis right
plot(p1,pq2,'ro','MarkerSize',8,'LineWidth',1.5)
ylabel('Elongation \epsilon [-]')
xlabel('Pressure [kPa]','FontSize',12); ylabel('Elongation [-]','FontSize',12)
legend('Curvature \kappa [1/m]','Elongation \epsilon [-]','FontSize',12,'Location','southeast')

% figure(5)
% plot(p1,pq1((length(pq1)/2)+1:end),'bo','MarkerSize',8,'LineWidth',1.5) % only half is simulated, the rest is assumed
% hold on;grid on; box on;
% plot(p1,q1_range,'r','LineWidth',1.5)
% xlabel('Moment [Nm]','FontSize',12); ylabel('Curvature \kappa [1/mm]','FontSize',12)
% legend('FEA with input mapping','Hyper-elastic stiffness model fit','FontSize',12,'Location','southeast')
% xlim([-0.2 0.2])
