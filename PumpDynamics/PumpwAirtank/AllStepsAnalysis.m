clear all;close all;clc;


T = [];
P = [];
U = [];
V = [];

Vsteps = [4:9];
PWC = 2;  % adapted PWC does not result in significant improvement
for ii = 1:length(Vsteps)
   
        volt = Vsteps(ii);
        file_name = ['Step',num2str(volt),'V','.txt'];
        data = load(file_name);
        % analyse raw data
        t = data(:,1);
        dp = data(:,3)*0.1;
        u = data(:,5);
        v = u./341.3333;
        
        figure(10)
        plot(t,dp,'LineWidth',1)
        hold on; grid on;
        xlabel('Time [s]');ylabel('Pressure Difference [kPa]')
        title('Step w/ airtank')      
        
        % analyse step input
      
        idx = find(u~=0); % 
        u = u(idx);
        t0 = t(idx(1)-idx(1)+1:idx(end)-idx(1)+1);
        t = t(idx);        
        dp =dp(idx);
        v = v(idx);
        
        
        T = [T;t];
        P = [P;dp];
        U = [U;u];
        V = [V;v];


        figure(PWC)
        plot(t0,dp)       
        hold on;grid on;
        xlabel('Time [s]');ylabel('Pressure Difference [kPa]')
        title('Step w/ airtank')      
%         plot(t,u)

end

%% Optimization

if PWC == 1
    % Linear
    x0 = rand(2,1);
    [bestxV,fval] = fmincon(@(x) ePressure(x,T,P,V),x0,[],[],[],[],[0;0],[Inf;Inf]);
    tau = bestxV(1);
    K = bestxV(2);
    for ii = 1:length(Vsteps)
  
        V = Vsteps(ii);

        [t,y] = ode45(@(t,x) pressureODE(t,x,tau,K,V),[1:1e-3:41],0);

        figure(PWC)
        plot(t,y)
        hold on;
        title('Linear Model')

    end

elseif PWC == 2
    % PWC
    x0 = rand(3,1);
    [bestxV,fval] = fmincon(@(x) ePressurePWC(x,T,P,V),x0,[],[],[],[],[0;0;0],[Inf;Inf;Inf]);
    fval
    tau = bestxV(1);
    alpha = bestxV(2);
    beta = bestxV(3);
    for ii = 1:length(Vsteps)
  
        V = Vsteps(ii);

        [t,y] = ode45(@(t,x) pressureODEPWC(t,x,tau,alpha,beta,V),[1:1e-3:41],0);

        figure(PWC)
        plot(t,y)
        hold on;
        title('PWC')
        
    end
    
elseif PWC == 3
    % adapted PWC
    x0 = rand(4,1);
    [bestxV,fval] = fmincon(@(x) ePressurePWCadapted(x,T,P,V),x0,[],[],[],[],[-Inf;-Inf;-Inf;-Inf],[Inf;Inf;Inf;Inf]);
    gamma = bestxV(1);
    delta = bestxV(2);
    alpha = bestxV(3);
    beta = bestxV(4);
    for ii = 1:length(Vsteps)
  
        V = Vsteps(ii);

        [t,y] = ode45(@(t,x) pressureODEPWCadapted(t,x,gamma,delta,alpha,beta,V),[1:1e-3:41],0);

        figure(PWC)
        plot(t,y)
        hold on;
        title('Adapted PWC')

    end
    
end

% figure(1)
% xlabel('Time [s]','FontSize',12);ylabel(' Pressure difference [kPa]','FontSize',12)
% legend('4V','5V','6V','7V','8V','9V','FontSize',12)
% legend('1V','2V','3V','4V','5V','6V','7V','8V','9V','FontSize',12)


figure(PWC)
xlabel('Time [s]','FontSize',12);ylabel(' Pressure difference [kPa]','FontSize',12)
legend('4V','5V','6V','7V','8V','9V','FontSize',12)



function dxdt = pressureODE(t,x,tau,K,V)

        dxdt = (-1/tau) * x + (K/tau) * V;

 end
function dxdt = pressureODEPWC(t,x,tau,alpha,beta,V)

Vmax = 9;
dV = 4;

if V <= dV
    K = 0;
elseif dV < V &&  V < Vmax 
    K = alpha*V + beta;
elseif V >= Vmax
    K = alpha*Vmax +beta;
end


dxdt = (-1/tau) * x + (K/tau) * V;

end

function dxdt = pressureODEPWCadapted(t,x,gamma,delta,alpha,beta,V)

Vmax = 9;
dV = 3;

tau = gamma*V + delta 

if V <= dV
    K = 0;
elseif dV < V < Vmax 
    K = alpha*V + beta;
elseif V > Vmax
   K = alpha*Vmax +beta;
end


dxdt = (-1/tau) * x + (K/tau) * V;

end




