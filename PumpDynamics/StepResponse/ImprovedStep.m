clear all;close all;clc;
global V;


sample_delay = 3;

T = [];
P = [];
U = [];


for ii =4:1:6
   
    for jj = 1:3

        file_name = [num2str(ii),'V',num2str(jj),'.txt'];
        data = load(file_name);
        
        t = data(1:end-sample_delay,1);
        p = data(sample_delay+1:end,3)*0.1;
        u = data(sample_delay+1:end,5);
        v = u/341.33333;
        
        T = [T;t];
        P = [P;p];
        U = [U;u];
     
        figure(1)
        plot(t,p)
        hold on;grid on;
       
    end
end

V_set = [4,5,6];

for ii = 1:3
  V = V_set(ii);  
[t,y] = ode45(@ODE,[0:1e-3:2.5],0);

plot(t,y)
hold on;

end

function dxdt = ODE(t,x)
global V;
tau_s= 0.094485;


if V < 2.5
    K = 4.48;
else
    K = 1.258*V + 4.48;
end

dxdt = (-1/tau_s) * x + (K/tau_s) * V;

end
