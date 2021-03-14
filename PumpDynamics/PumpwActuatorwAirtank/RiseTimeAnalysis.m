clear all;close all;clc;

T = [];
P = [];
U = [];
V = [];

dV = 1;
% Vmax = 8;

Vsteps = [4:11];

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
        [ d, ix ] = min( abs( dp-(0.63*dp(end)) ) );
        tau_approx(ii) =t0(ix);
        
        
        T = [T;t];
        P = [P;dp];
        U = [U;u];
        V = [V;v];

        
        figure(2)
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
    
%     x0 = 10*rand(5,1);
%     x0 = [-0.17,5,0.72,2.56,5];
    x0 = [   -0.16 ;   4.12 ;   0.7146;   2.5936 ;   5.6049];
    options = optimoptions('fmincon','OptimalityTolerance',1e-12,'FunctionTolerance',1e-10,'StepTolerance',1e-12,'ConstraintTolerance',1e-12);
    [bestxV,fval,exitflag] = fmincon(@(x) ePressurePWCtau(x,T,P,V),x0,[],[],[],[],[-0.25;0;0.4;0;dV],[0;10;0.8;12;12],[],options);
%     [bestxV,fval] = fmincon(@(x) ePressurePWC(x,T,P,V),x0,[],[],[],[],[],[],[],options);
    fval
    bestxV
    mu = bestxV(1);
    nu = bestxV(2);
    alpha = bestxV(3);
    beta = bestxV(4);
    Vmax = bestxV(5);
    
  
%     
Volts = [4;5;6;7;8;9;10;11];
taus  = [4.5838;4.5263;4.5795;4.1879;4.0271;3.8447;3.6373;3.4403];        
    
    
    
    
    for ii = 1:length(Vsteps)
  
        V = Vsteps(ii);

        [t,y] = ode45(@(t,x) pressureODEPWC(t,x,mu,nu,alpha,beta,V,dV,Vmax),[0:1e-3:25],0);

        figure(2)
        plot(t,y,'LineWidth',1)
        hold on;
%         title('PWC')
%         legend('4V',5V )
        axis([0 25 0 80])
    end
% 
% 
% 
% 
function dxdt = pressureODEPWC(t,x,mu,nu,alpha,beta,V,dV,Vmax)

tau = -0.16*V + 4.12;
% tau = mu*V + nu;

if V <= dV
    K = 0;
elseif dV < V &&  V <= Vmax 
    K = alpha*V + beta;
elseif V > Vmax
    K = alpha*Vmax +beta;
end


dxdt = (-1/tau) * x + (K/tau) * V;

end


