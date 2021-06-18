clear all;close all;clc;

%pressure model fit

%% Before loading check refence input in bottom V(t) = off + amp*sin(2pift)
data = load('step711V.txt'); 

% freq = 1;
% freq = 0.5;
freq = 0.25;
% freq = 0.1;
% freq =  0.05;



t = data(:,1);
p = (data(:,2)-data(1,3))*0.1;
u = data(:,6);
v = u/341.3333;


figure(1)
yyaxis left
plot(t,p)
hold on; grid on;

yyaxis right
plot(t,u)
xlabel('Time [s]')

% time = t(1:(length(t)-2000));
% pres = p(2001:end);
% inpt = v(2001:end);

time = t;
pres = p;
inpt = v;


figure(2)
yyaxis left
plot(time,pres)
hold on; grid on;

yyaxis right
plot(time,inpt)
xlabel('Time [s]')
options = optimoptions('fmincon','Display','iter','ConstraintTolerance',1e-12,'OptimalityTolerance',1e-8,'StepTolerance',1e-12);
[lambda] =  fmincon(@(lambda) odePressureE(lambda,time,pres,freq),[0.23;2.23],[],[],[],[],[0;0],[5;5],[],options);


% plot solution

[t,p] = ode45(@(t,p) verfication(t,p,lambda,freq),time,0);



figure(3)
plot(time,pres)
hold on;grid on;
plot(t,p)
xlabel('Time [s]','FontSize',12)
ylabel('Pressure [kPa]','FontSize',12)
legend('Measurement','Fit','FontSize',12)
ylim([0 80])

% figure(4)
% load('step9V.txt')
% t = data(:,1);
% p = (data(:,2)-data(1,3))*0.1;
% u = data(:,6);
% plot(t,p)
% hold on;grid on;



% % 
function E = odePressureE(lambda,time,pres,freq)

l1 = lambda(1);
l2 = lambda(2);

[t,p] = ode45(@(t,p) pmodel(t,p,freq,l1,l2),time,0);

E = norm(pres - p)^2



end


function dpdt = pmodel(t,p,freq,l1,l2)


dpdt = -l1*p + l2*(9+2*sin(2*pi*freq*t));

end


function dpdt = verfication(t,p,lambda,freq)


dpdt = -lambda(1)*p + lambda(2)*(9+2*sin(2*pi*freq*t));

end