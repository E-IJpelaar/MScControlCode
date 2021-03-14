clear all;close all;clc;

sample_delay = 3;

params = cell(5,3);

for ii = 2:6
   
    for jj = 1:3

        file_name = [num2str(ii),'V',num2str(jj),'.txt'];
        data = load(file_name);
        t = data(1:end-sample_delay,1);
        p = data(sample_delay+1:end,3)*0.1;
        U = data(sample_delay+1:end,5);
        u = U(1,1);

        x0 = rand(2,1);
        bestx = fminsearch(@(x) errorPressure(x,t,p,u),x0);
    
        params{ii-1,jj} = bestx;
    
    end
end


figure(1)
plot(t,p)
hold on;grid on;
plot(t,(u*bestx(1)*(1-exp(-t/bestx(2)))))
