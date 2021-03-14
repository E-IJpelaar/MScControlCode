clear all;close all;clc;

% First load in data and determine best rise-time characteristics
%% Load all data and analyse approximate tau and K 

Vsteps = [2:12]; % all data

tau_approx = zeros(1,length(Vsteps));
K_approx = zeros(1,length(Vsteps));
for ii = 1:length(Vsteps)
   
        volt = Vsteps(ii);
        file_name = ['Step',num2str(volt),'V','.txt'];
        data = load(file_name);
        % analyse raw data
        t = data(:,1);
        dp = data(:,3)*0.1;
        u = data(:,5);
        v = u./341.3333;
        
%         figure(1)
%         plot(t,dp,'LineWidth',1)
%         hold on; grid on;
%         xlabel('Time [s]');ylabel('Pressure Difference [kPa]')  
%         title('Step w/ airtank & actuator')
%         
        % analyse step input
      
        idx = find(u~=0); % 
        u = u(idx);
        t0 = t((idx(1)-idx(1)+1):(idx(end)-idx(1)+1));
        t = t(idx); 
        dp =dp(idx);
        v = v(idx);
        [ d, ix ] = min( abs( dp-(0.63*dp(end)) ) );
        tau_approx(ii) =t0(ix);
        K_approx(ii) = mean(dp(end-500:end))/v(end);
        
             
        figure(2)
        plot(t0,dp)       
        hold on;grid on;
        xlabel('Time [s]');ylabel('Pressure Difference [kPa]')
        title('Step w/ airtank & actuator')

end



figure(3)
plot(Vsteps,tau_approx,'ro')
hold on;grid on;
xlabel('Volt [V]');ylabel('Approximate Rise time \tau [1/s]')

figure(4)
plot(Vsteps,K_approx,'bx')
hold on;grid on;
xlabel('Volt [V]');ylabel('Approximate K [kPa/V]')

%% Only consider data used for optimisation

T = [];
P = [];
U = [];
V = [];

Vsteps = [2:12];   % removed outliers
dV = 1; % with outlier correction
    

for ii = 1:length(Vsteps)
   
        volt = Vsteps(ii);
        file_name = ['Step',num2str(volt),'V','.txt'];
        data = load(file_name);
        
        % analyse raw data
        t = data(:,1);
        dp = data(:,3)*0.1;
        u = data(:,5);
        v = u./341.3333;

        idx = find(u~=0); % 
        u = u(idx);
        t0 = t((idx(1)-idx(1)+1):(idx(end)-idx(1)+1));
        t = t(idx); 
        dp =dp(idx);
        v = v(idx);
        
        
        T = [T;t];
        P = [P;dp];
        U = [U;u];
        V = [V;v];

        
        figure(5)
        plot(t0,dp)       
        hold on;grid on;
        xlabel('Time [s]');ylabel('Pressure Difference [kPa]')
        title('Step w/ airtank & actuator')

end

% From the graphs one would suggest to fit K based on all measured
% instances and fit tau between [4:12] V


%% Optimalisation over PWC function alpha,beta,Vmax

x0 = [3; 0.7146;   2.5936 ;   5.6049];
options = optimoptions('fmincon','OptimalityTolerance',1e-14,'FunctionTolerance',1e-14,'StepTolerance',1e-14,'ConstraintTolerance',1e-14,'MaxFunctionEvaluations',1e5);
[bestxV,fval,exitflag] = fmincon(@(x) ePressurePWC(x,T,P,V),x0,[],[],[],[],[0;0;-5;dV],[Inf;2;12;12],[],options);   
fval
bestxV
tau = bestxV(1);
alpha = bestxV(2);
beta = bestxV(3);
Vmax = bestxV(4);
    


%% Plot obtained solution

for ii = 1:length(Vsteps)
  
        V = Vsteps(ii);

        [t,y] = ode45(@(t,x) pressureODEPWC(t,x,tau,alpha,beta,V,dV,Vmax),[0:1e-3:25],0);

        figure(5)
        plot(t,y,'LineWidth',1)
        hold on;
        axis([0 25 0 80])
    end
% 
% 
% 
% 
function dxdt = pressureODEPWC(t,x,tau,alpha,beta,V,dV,Vmax)

if  V <= Vmax 
        K = alpha*V + beta;
elseif V > Vmax
        K = alpha*Vmax +beta;
end


dxdt = (-1/tau) * x + (K/tau) * V;

end




