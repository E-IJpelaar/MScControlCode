clear all;close all;clc;
%% System parameters
L_act = 64.5;                  % [mm] actuator length
Nmode = 1;                     % Nmode used in IKFEMsolver2DOF script        

%% Load Force data (corresponding to pulling the top plate)
load ForceData_20201123T171150.mat % load data 
force_data = sortrows(data.',1).'; % sort data by increasing Force

                  
for kk = 1:2:9           % FEM simulations included 2.5,7.5,12.5,17.5 N however, in data they are displayed as 2,7,12,17 so 0.5 should be added
force_data(1,kk) = force_data(1,kk)+0.5;
end
force_data = [zeros(4,1),force_data];  % add an additional point 0 N = 0 elongation
F = force_data(1,:);                   % Force vector
f_q1 = force_data(2,:);                % q1 of force vector (curvature)
f_q2 = force_data(3,:);                % q2 of force vector (elongation)

%% Load Pressure data (corresponding to pressurized bellows)
load PresureElong_20201123T161418.mat % load data 
pres_data = sortrows(data.',1).'; % sort data by increasing Force

pres_data = [zeros(4,1),pres_data];        % add an additional point 0 N = 0 elongation

p = pres_data(1,:);
p_q1 = pres_data(2,:);                % q1 of pressure vector (curvature)
p_q2 = pres_data(3,:);                % q2 of pressure vector (elongation)

%% Plot q1 and q2 for both pressure and force
% say that the force applied to the top is deemed valid

figure(1)
yyaxis left
plot(p_q1,p,'bx')
hold on; grid on;box on;
plot(p_q2,p,'bo')
ylabel('Pressure [kPa]','FontSize',12)

yyaxis right
plot(f_q1,F,'rx')
plot(f_q2,F,'ro')
ylabel('Force [N]','FontSize',12)
legend('Elongation analysis \kappa','Elongation analysis \epsilon','Force analysis \kappa ','Force analysis \epsilon','FontSize',12)
xlabel('Modal coordinate [-]','FontSize',12)

%% Find the pressure to force mapping

fun = @(x)mappingForce(x,p,F);
x0 = 1;
[alpha,~,exitflag]= fmincon(fun,x0,[],[],[],[],[],[])

%% Create mapping matrix H

H = zeros(2,2);
H(1,1) = (alpha/2);
H(1,2) = H(1,1);

p1 = p;                             % For the elongation both p1 and p2 are equal
p2 = p;

F_mapped = H(1,:)*[p1;p2];  % Mapped Force (kPa to N)


% assume negative pressure causes equal elongation in opposite direction (no contact/interaction)
F_mapped = [-sort(F_mapped),F_mapped]; 
p_q2 = [-sort(p_q2),p_q2];

% fit hyper-elastic stiffness model to q

options = optimoptions('fmincon','Display','iter','MaxFunctionEvaluations',1e4 );
fun = @(x)stiffnessModel(x,p_q2,F_mapped);
x0 = [2500 2500 1];
[beta,~,exitflag] = fmincon(fun,x0,[],[],[],[],[0;0;0],[],@nonlcon,options)

q2s = linspace(-0.6,0.6,100);

F_fit = (beta(1) + beta(2).*((tanh(beta(3).*q2s)).^2 -1)).*q2s;

figure(2)
plot(F_mapped((length(p_q2)/2)+1:end),p_q2((length(p_q2)/2)+1:end),'bo','MarkerSize',8,'LineWidth',1.5)
hold on;grid on; box on;
plot(F_fit,q2s,'r','LineWidth',1.5)
xlabel('Force [N]','FontSize',12); ylabel('Elongation q_\epsilon [-]','FontSize',12)
legend('FEA with input mapping','Hyper-elastic stiffness model fit','FontSize',12)

% elongation = p_q2 *L_act;
figure(3)
plot(F,f_q2,'bo','MarkerSize',8,'LineWidth',1.5)
grid on; hold on;
plot(alpha.*p,p_q2((length(p_q2)/2) +1:end),'rx','MarkerSize',8,'LineWidth',1.5)
legend('Force analysis','Force with input mapping','FontSize',12)
xlabel('Force [N]','FontSize',12);ylabel('Elongation q_\epsilon [-]','FontSize',12)


figure(4)
% plot(p,1./(pres_data(2,:)*L_act),'o','MarkerSize',6) % this plot radius (r = 1/k [mm])
plot(p,(pres_data(2,:)*L_act),'o','MarkerSize',6)
hold on; grid on;
plot(p,pres_data(3,:)*L_act,'o','MarkerSize',6)
xlabel('Pressure [kPa]','FontSize',12);ylabel('Output [-]','FontSize',12)
legend('Curvature \kappa [1/mm]','Elongation \epsilon [mm]','FontSize',12)


