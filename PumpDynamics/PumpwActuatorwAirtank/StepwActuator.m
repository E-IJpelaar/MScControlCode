clear all;close all;clc;

T = [];
P = [];
U = [];
V = [];

dV = 1;
% Vmax = 8;

Vsteps = [2:12];

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
        
        % analyse step input
      
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


        figure(4)
        plot(t0,dp)       
        hold on;grid on;
        xlabel('Time [s]');ylabel('Pressure Difference [kPa]')
        title('Step w/ airtank & actuator')
%         plot(t,u)

end

% 
% 
% 
    % PWC
    x0 = [3.8499;0.7171;2.5607;5.6347];
%     x0 = rand(4,1);
    options = optimoptions('fmincon','OptimalityTolerance',1e-12,'FunctionTolerance',1e-10,'StepTolerance',1e-12,'ConstraintTolerance',1e-12);
    [bestxV,fval,exitflag] = fmincon(@(x) ePressurePWC(x,T,P,V),x0,[],[],[],[],[0;0;-Inf;dV],[Inf;Inf;Inf;12],[],options);
%     [bestxV,fval] = fmincon(@(x) ePressurePWC(x,T,P,V),x0,[],[],[],[],[],[],[],options);
    fval
    tau = bestxV(1);
    alpha = bestxV(2);
    beta = bestxV(3);
    Vmax = bestxV(4);
    
    
    for ii = 1:length(Vsteps)
  
        V = Vsteps(ii);

        [t,y] = ode45(@(t,x) pressureODEPWC(t,x,tau,alpha,beta,V,dV,Vmax),[0:1e-3:25],0);

        figure(4)
        plot(t,y,'LineWidth',1)
        hold on;
%         title('PWC')
%         legend('4V',5V )
% %         axis([0 25 0 80])
    end
% 
% 
% 
% 
function dxdt = pressureODEPWC(t,x,tau,alpha,beta,V,dV,Vmax)


if V <= dV
    K = 0;
elseif dV < V &&  V <= Vmax 
    K = alpha*V + beta;
elseif V > Vmax
    K = alpha*Vmax +beta;
end


dxdt = (-1/tau) * x + (K/tau) * V;

end