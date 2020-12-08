clear all;close all;clc;

mm2m = 1e-3;                   % mm to m
L_act = 64.5*mm2m;             % [m] actuator length
Nmode = 1;                     % Nmode used in IKFEMsolver2DOF script        
r = 12.56*mm2m;                % [m] lever on which pressure acts
%% Evaluate Force experiments
load ForceData_20201208T151754.mat  % load .mat file
dataf = sortrows(data.',1).';       % sort data by increasing Force
dataf = [zeros(4,1),dataf];         % add 0 N = 0 elongation point

% assign data
F = dataf(1,:);                     % Force is in first row
for kk = 2:2:7                      % Experiments were conducted for decimal pressures, but are not saved that way. So we add them manually to the correct data
F(kk) = F(kk)+0.5;
end
q1f= dataf(2:2+(Nmode-1),:);        % curvatures
q2f= dataf(2+Nmode:1+2*Nmode,:);    % elongation

clearvars data
%% Evaluate Pressure experiments
load PressureElong_20201208T150920.mat
datap = sortrows(data.',1).';   %   sort data by increasing pressure
datap = [zeros(4,1),datap];     %   add 0 Pa = 0 elongation point

% assign data
p1 = datap(1,:);                % pressure in both bellows is equal
p2 = datap(1,:);                % pressure in both bellows is equal
q1p= datap(2:2+(Nmode-1),:);    % curvatures
q2p= datap(2+Nmode:1+2*Nmode,:);% elongation


%% First plot

figure(1)
yyaxis left
plot(-q1p,p1,'bx','MarkerSize',6,'LineWidth',1.5)
hold on; grid on;
plot(q2p,p1,'bo','MarkerSize',6,'LineWidth',1.5)
ylabel('Pressure [kPa]','FontSize',12)

yyaxis right
plot(-q1p,F,'rx','MarkerSize',6,'LineWidth',1.5)
plot(q2p,F,'ro','MarkerSize',6,'LineWidth',1.5)
ylabel('Force [N]','FontSize',12)
xlabel('Modal coordinate [-]','FontSize',12)
legend('Elongation analysis \kappa','Elongation analysis \epsilon','Force analysis \kappa','Force analysis \epsilon','FontSize',12,'Location','southeast')

%% Find input mapping
fun = @(x)mappingForce(x,p1,F);
x0 = 1;
[alpha,~,exitflag]= fmincon(fun,x0);

% Mapping matrix H
H = [alpha  ,  alpha;
     alpha*r, -alpha*r];
disp('H = ');
disp(num2str(H))

% Result with mapping
figure(2)
plot(2*alpha*p1,q2p,'bo','MarkerSize',6,'LineWidth',1.5)
hold on; grid on;
plot(F,q2f,'rx','MarkerSize',6,'LineWidth',1.5)
ylabel('Elongation \epsilon [-]','FontSize',12);xlabel('Force [N]','FontSize',12)
legend('Force with input mapping','Force analysis','FontSize',12,'Location','southeast')

save('mapping.mat','H','datap')
