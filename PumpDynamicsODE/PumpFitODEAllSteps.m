clear all;close all;clc;

%pressure model fit
freq_vec = [0.5;0.25;0.1;0.05];
freq_name_vec = [{'05'},{'025'},{'01'},{'005'}];


[lambda] =  fmincon(@(lambda) odePressureE(lambda,freq_vec,freq_name_vec),[1;1],[],[]);


% plot solution

[t,p] = ode45(@(t,p) verfication(t,p,lambda,freq),time,0);



figure(3)
plot(time,pres)
hold on;grid on;
plot(t,p)
xlabel('Time [s]')
ylabel('Pressure [kPa]')
legend('Measurement','Fit')
ylim([0 80])


% % 
function E = odePressureE(lambda,freq_vec,freq_name_vec)

l1 = lambda(1);
l2 = lambda(2);

P = [];
Pexp = [];

for ii = 1:length(freq_vec)

freq = freq_vec(ii);

filename = (['sinf', freq_name_vec{ii},'.txt']);
data = load(filename); 

t = data(:,1);
p = data(:,4)*0.1;    
time = t;
pres = p;

Pexp = [Pexp;pres];


[t,p] = ode45(@(t,p) pmodel(t,p,freq,l1,l2),time,0);

P = [P;p];

end

E = norm(Pexp - P)^2;

end



function dpdt = pmodel(t,p,freq,l1,l2)


dpdt = -l1*p + l2*(7+2*sin(2*pi*freq*t));

end


function dpdt = verfication(t,p,lambda,freq)


dpdt = -lambda(1)*p + lambda(2)*(7+2*sin(2*pi*freq*t));

end