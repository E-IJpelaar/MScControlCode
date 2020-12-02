clear all;close all;clc;
%% System parameters
L_act = 64.5;                  % [mm] actuator length
Nmode = 1;                     % Nmode used in IKFEMsolver2DOF script        

%% Load Pressure data (corresponding to pressurized bellows)
load PressureRot_20201123T192849.mat % load data 
pres_data = sortrows(data.',1).'; % sort data by increasing Force

pres_data = [zeros(4,1),pres_data];        % add an additional point 0 N = 0 elongation

p = pres_data(1,:);
p_q1 = -pres_data(2,:);               % q1 of pressure vector (curvature) q1 = -q1 assumption
p_q2 = pres_data(3,:);                % q2 of pressure vector (elongation)

%% Plot q1 and q2 for both pressure and force
% say that the force applied to the top is deemed valid

figure(1)
plot(p_q1,p,'bx')
hold on; grid on;box on;
plot(p_q2,p,'bo')
ylabel('Pressure [kPa]')
xlabel('Modal coordinate [-]')
legend('Curvature','Elongation')

%% Fit rotation to simulation
alpha = 0.146171313597241;
p1 = p;                             % For the rotation one bellow is pressurized
p2 = zeros(1,length(p1));    

H(2,1) = alpha*12.56e-3;
H(2,2) = -alpha*12.56e-3;

M_mapped = H(2,:)*[p1;p2];
M_mapped = [-M_mapped,M_mapped];
p_q1 = [-p_q1,p_q1];

options = optimoptions('fmincon','Display','iter','MaxFunctionEvaluations',1e4 );
fun = @(x)stiffnessModel(x,p_q1,M_mapped);
x0 = [2300.86746927188,2300.84669008190,0.00741771829813126];
[gamma,~,exitflag] = fmincon(fun,x0,[],[],[],[],[0;0;0],[],@nonlcon,options)


q1s = linspace(-1.2,1.2,100);
M_fit = (gamma(1) + gamma(2).*((tanh(gamma(3).*q1s)).^2 -1)).*q1s;


figure(2)
plot(M_mapped((length(p_q1)/2)+1:end),p_q1((length(p_q1)/2)+1:end),'bo','MarkerSize',8,'LineWidth',1.5)
hold on;grid on;
plot(M_fit,q1s,'r','LineWidth',1.5)
xlabel('Moment [Nm]','FontSize',12); ylabel('Curvature q_\kappa [-]','FontSize',12)
legend('FEA with input mapping','Hyper-elastic stiffness model fit','FontSize',12)
axis([-0.2 0.2 -1.5 1.5])

%% Elongation part in Rotation



H(1,1) = alpha;
H(1,2) = H(1,1);
F_mapped = H(1,:) *[p1;p2];
% 
p_q1 = -pres_data(2,:); %only get half again
beta = [1.1579,1.1423,0.0003]*1.0e+03 ;


F_fit = (1  + 0.8788*p_q1).*((beta(1) + beta(2).*((tanh(beta(3).*p_q2)).^2 -1)).*p_q2);  % Force according to Pressure2Force Fit

fun = @(x)(sum((1  + x.*p_q1).*((beta(1) + beta(2).*((tanh(beta(3).*p_q2)).^2 -1)).*p_q2)-F_mapped).^2);
x0 = 0.5;
x = fmincon(fun,x0)

figure(3)
plot(F_mapped,p_q2,'bo')
hold on; grid on; box on;
plot(F_fit,p_q2,'rx')
xlabel('Force [N]');ylabel('Modal Coordinate [-]')
legend('Elongation in rotation experiment','Elongation as for Fit')
% 


figure(4)
% plot(p,1./(pres_data(2,:)*L_act),'o','MarkerSize',6) % this plot radius (r = 1/k [mm])
plot(p,-(pres_data(2,:)*L_act),'o','MarkerSize',6)
hold on; grid on;
plot(p,pres_data(3,:)*L_act,'o','MarkerSize',6)
xlabel('Pressure [kPa]','FontSize',12);ylabel('Output [-]','FontSize',12)
legend('Curvature \kappa [1/mm]','Elongation \epsilon [mm]','FontSize',12)


gamma = [2300.86746927188,2300.84669008190,0.00741771829813126];
K =  (gamma(1) + gamma(2).*((tanh(gamma(3).*q1s)).^2 -1));
figure(6)
plot(q1s,K)
