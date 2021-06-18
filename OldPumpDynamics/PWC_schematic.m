clear all;close all;clc;

Volts = linspace(0,12,1000);
dV = 3;
Vmax = 9;
alpha = 0.2961;
beta = 6.8683;

for ii = 1:length(Volts)
    V = Volts(ii);
if V <= dV
    K(ii) = 0;
elseif dV < V && V < Vmax 
    K(ii) = alpha*V + beta;
elseif V > Vmax
    K(ii) = alpha*Vmax +beta;
end

end

figure(1)
plot(Volts,K,'LineWidth',1.5)
hold on;grid on;
yline(K(end),'r-.','LineWidth',1.5)
xline(Vmax,'k-.','LineWidth',1.5)
xlabel('Electric potential [V]','Interpreter','Latex');ylabel('K(V) [kPa/V]','Interpreter','Latex')
text(3.2,3,"$\partial V$",'Interpreter','Latex')
text(5,8,"$\alpha V + \beta$",'Interpreter','Latex')
text(10,9.75,"$K_{max}$",'Interpreter','Latex')
text(9.2,0.5,"$V_{max}$",'Interpreter','Latex')
text(0.7,0.2,"$deadzone$",'Interpreter','Latex')