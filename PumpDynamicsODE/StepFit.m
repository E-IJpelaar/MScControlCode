clear all;close all;clc;

clear all;close all;clc;

%pressure model fit

%% Before loading check refence input in bottom V(t) = off + amp*sin(2pift)
data = load('160611V(1).txt'); 
Vstep = 11;

t = data(:,1);

%% For fitting p1
% p = (data(:,2)-data(1,3))*0.1;
% u = data(:,6);
% t2 = find(t>2,1);
% t22 = find(t>22,1);
% t = t(t2:t22);
% p = p(t2:t22);
% u = u(t2:t22);

%% For fitting p2
p = (data(:,4)-data(1,5))*0.1;
u = data(:,7);
t62 = find(t>62,1);
t82 = find(t>82,1);
t = t(t62:t82);
p = p(t62:t82);
u = u(t62:t82);


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
[lambda] =  fmincon(@(lambda) odePressureE(lambda,time,pres,Vstep),[0.23;2.23],[],[],[],[],[0;0],[5;5],[],options);


% plot solution

[t,p] = ode45(@(t,p) verfication(t,p,lambda,Vstep),time,0);



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
function E = odePressureE(lambda,time,pres,Vstep)

l1 = lambda(1);
l2 = lambda(2);

[t,p] = ode45(@(t,p) pmodel(t,p,l1,l2,Vstep),time,0);

E = norm(pres - p)^2



end


function dpdt = pmodel(t,p,l1,l2,Vstep)

% if t < 2
%     u = 0;
% else 
    u = Vstep;
% end

dpdt = -l1*p + l2*u;

end


function dpdt = verfication(t,p,lambda,Vstep)

% if t < 2
%     u = 0;
% else 
    u = Vstep;
% end

dpdt = -lambda(1)*p + lambda(2)*u;

end