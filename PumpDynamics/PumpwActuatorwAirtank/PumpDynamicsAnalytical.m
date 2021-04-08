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
        [ d, ix ] = min(abs( dp-(0.632*dp(end)) ) );
        tau_approx(ii) =t0(ix);
        K_approx(ii) = mean(dp(end-500:end))/v(end); % mean pressure last 5 seconds
        
             
        figure(2)
        plot(t0,dp,'LineWidth',1)       
        hold on;grid on;
        xlabel('Time [s]');ylabel('Pressure [kPa]')
        %title('Step w/ airtank & actuator')

end

figure(2)
% legend('2V','3V','4V','5V','6V','7V','8V','9V','10V','11V','12V','Orientation','horizontal','NumColumns',6,'Location','south')


figure(3)
plot(Vsteps,tau_approx,'bx','MarkerSize',14,'LineWidth',2)
hold on;grid on;
xlabel('Volt [V]');ylabel('Time constant \tau [1/s]')

figure(4)
plot(Vsteps,K_approx,'bx','MarkerSize',14,'LineWidth',2)
hold on;grid on;
xlabel('Volt [V]');ylabel('Constant K [kPa/V]')


% first find best PWC solution for K

x0 = [2,-1,4.4];
options = optimoptions('fmincon','OptimalityTolerance',1e-14,'FunctionTolerance',1e-14,'StepTolerance',1e-14,'ConstraintTolerance',1e-14,'MaxFunctionEvaluations',1e5);
[bestxV,fval,exitflag] = fmincon(@(x) eK(x,Vsteps,K_approx),x0,[],[],[],[],[0;-5;0],[5;12;12],[],options);   

alpha = bestxV(1);
beta = bestxV(2);
Vmax= bestxV(3);

K_poly = polyfit(Vsteps,K_approx,4);

for ii = 1:length(Vsteps)
    
    V = Vsteps(ii);
    K_fit(ii) = funcK(V,alpha,beta,Vmax);
       
end

% plot solution for K
figure(4)
% plot(Vsteps,K_fit)
hold on;grid on;
plot(linspace(2,12,1000),polyval(K_poly,linspace(2,12,1000)),'r','LineWidth',1.5)
legend('Experimental estimate','4^{th}-order polynomial fit','FontSize',12)
text(4.5,2.2,['K(V) = ',num2str(K_poly(1),2),'V^4 + ',num2str(K_poly(2),2),'V^3 + ',num2str(K_poly(3),2),'V^2 + ',num2str(K_poly(4),2),'V + ',num2str(K_poly(5),2)])


%% Optimization tau analytical
Vtau = [4:12]; % Volts used to estimate parameter tau
tau_approx = tau_approx(3:end);

P = polyfit(Vtau,tau_approx,1);
tau_fit = polyval(P,linspace(2,12,1000));

figure(3)
% plot(Vtau,tau_approx);
hold on; grid on;
plot(linspace(2,12,1000),tau_fit,'r','LineWidth',1.5)
text(8.2,3.5,['\tau(V) = ',num2str(P(1),2),'V + ',num2str(P(2),2)])
legend('Experimental estimate','1^{st}-order linear fit','FontSize',12)

mu = P(1);
nu = P(2);


%% We use FMINCON to determine PWC function. Rise time is determined analytically
% otherwise we get surface area compensation (area errror) using fmincon when determining
% the rise time

%% Plot solution

for ii = 1:length(Vsteps)
  
        V = Vsteps(ii);

        [t,y] = ode45(@(t,x) pressureODEPWC(t,x,mu,nu,alpha,beta,V,1,Vmax,K_poly),[0:1e-3:25],0);

        figure(2)
        plot(t,y,'LineWidth',1)
        hold on;
        axis([0 25 0 80])
    end
%  
function dxdt = pressureODEPWC(t,x,mu,nu,alpha,beta,V,dV,Vmax,K_poly)

tau = mu*V + nu;

% PWC
% if V <= dV
%     K = 0;
% elseif dV < V &&  V <= Vmax 
%     K = alpha*V + beta;
% elseif V > Vmax
%     K = alpha*Vmax +beta;
% end

% Fourth order fit
K = polyval(K_poly,V);


dxdt = (-1/tau) * x + (K/tau) * V;

end













% %% Optimization of Tau with FMINCON
% T = [];
% P = [];
% U = [];
% V = [];
% 
% for ii = 1:length(Vtau)
%    
%         volt = Vtau(ii);
%         file_name = ['Step',num2str(volt),'V','.txt'];
%         data = load(file_name);
%         % analyse raw data
%         t = data(:,1);
%         dp = data(:,3)*0.1;
%         u = data(:,5);
%         v = u./341.3333;
%             
%         idx = find(u~=0); % 
%         u = u(idx);
%         t0 = t((idx(1)-idx(1)+1):(idx(end)-idx(1)+1));
%         t = t(idx); 
%         dp =dp(idx);
%         v = v(idx);
%        
%         T = [T;t];
%         P = [P;dp];
%         U = [U;u];
%         V = [V;v];
% 
%  
%         figure(10)
%         plot(t0,dp)       
%         hold on;grid on;
%         xlabel('Time [s]');ylabel('Pressure Difference [kPa]')
%         title('Step w/ airtank & actuator')
%    
% end
% 
% for ii = 1:length(V)
%     
%     v = V(ii);
%     K = funcK(v,alpha,beta,Vmax);
%     
% end
% 
% 
% % x0 = [mu_analytical, nu_analytical];
% x0 = [-0.0603    4.2905];
% options = optimoptions('fmincon','OptimalityTolerance',1e-14,'FunctionTolerance',1e-14,'StepTolerance',1e-14,'ConstraintTolerance',1e-14,'MaxFunctionEvaluations',1e5);
% [bestxV,fval,exitflag] = fmincon(@(x) ePressureKknown(x,P,V,K,T),x0,[],[],[],[],[-0.2,0],[-0.1,10],[],options);   
% mu = bestxV(1);
% nu = bestxV(2);
% bestxV
% 
% figure(5)
% plot(Vtau,mu.*Vtau + nu)
% 
%    
%     for ii = 1:length(Vsteps)
%   
%         V = Vsteps(ii);
% 
%         [t,y] = ode45(@(t,x) pressureODEPWC(t,x,mu,nu,alpha,beta,V,1,Vmax),[0:1e-3:25],0);
% 
%         figure(2)
%         plot(t,y,'LineWidth',1)
%         hold on;
% %         title('PWC')
% %         legend('4V',5V )
%         axis([0 25 0 80])
%     end
% % 
% % 
% % 
% % 
% function dxdt = pressureODEPWC(t,x,mu,nu,alpha,beta,V,dV,Vmax)
% 
% % tau = -0.16*V + 4.12;
% tau = mu*V + nu;
% 
% if V <= dV
%     K = 0;
% elseif dV < V &&  V <= Vmax 
%     K = alpha*V + beta;
% elseif V > Vmax
%     K = alpha*Vmax +beta;
% end
% 
% 
% dxdt = (-1/tau) * x + (K/tau) * V;
% 
% end
% 
% 
% 
% 
% 
% function eP = ePressureKknown(x,P,V,K,T)
% 
% mu = x(1);
% nu = x(2);
% 
% tau = mu.*V+nu;
% 
% 
% P_star = K.*(1-exp(-T./tau)).*V;
% 
% 
% 
% eP = sum((P - P_star(:)).^2);
% 
% 
% end
    




