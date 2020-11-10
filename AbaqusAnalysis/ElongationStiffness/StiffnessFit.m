clear all;close all;clc;
%% Conversions
kPa2Pa = 1e3;
mm22m2 = 1e-6;
mm2m   = 1e-3;

 %% Determine area on which force acts
A_eff = 0.0006; % m^2    % use this for now, eventually it doesn't matter how Force is mapped.
% 
 %% Pressure range
p =  [0; 5; 10; 20; 30; 40; 50; 60; 70; 80; 90; 100]*kPa2Pa; % Simulated Pressures
% 
 %% Elongation
e =  [0;  6.7059 ; 11.4679 ; 17.9628 ;  22.4614 ; 25.9270 ;  28.7604 ;  31.1654  ; 33.2600  ;  35.1187 ; 36.7918 ;38.3149]*mm2m; % simulated elongation output

%% Force Mapping
F_eff = p*A_eff;


%% Stiffness Fit
fun = @(x)stiffnessModel(x,e,F_eff);
x0 = [100 100 100];
[alpha,~,exitflag] = fmincon(fun,x0,[],[],[],[],[0;0;-Inf],[],@nonlcon)


%% Figures

eps = linspace(0,0.05,1000);
figure(2)
plot(e,F_eff,'o','MarkerSize',8,'MarkerFaceColor','b')
hold on; grid on; box on;
plot(eps,(alpha(1) + alpha(2).*((tanh(alpha(3).*eps)).^2 -1)).*eps)
xlabel('Elongation in y-direction [m]')
ylabel('F [N]')
legend('Simulated','Hyper-Elastic Model Fit')

figure(3)
plot(e,F_eff./e,'o','MarkerSize',8,'MarkerFaceColor','b')
hold on; grid on; box on;
plot(eps,alpha(1) + alpha(2).*((tanh(alpha(3).*eps)).^2 -1))
xlabel('Elongation in y-direction [m]')
ylabel('K_{yy} = F/\epsilon [N/m]')
legend('Simulated','Hyper-Elastic Model Fit')

