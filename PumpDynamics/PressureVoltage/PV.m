clear all;close all;clc;
addpath('C:\Users\s159261\OneDrive - TU Eindhoven\Documents\GitHub\MScControlCode\PressureVoltage\V4')
hPa2kPa = 0.1;

for ii = 1:13
    
    jj = ii-1;
    name = (['data', num2str(jj),'.txt']);
    data = load (name);
    data = data(101:end,:);
    
%     D = isnan(data);
%     [n,~] = find(D == 1);
%     data(n,:)=[];
%     
    p0 = data(1,2);
    dp  = data(:,3);
    p = data(:,4);
    pwm = data(:,5);
    angle = data(:,6);
    V = (12/4096)*pwm;
    
    avg_dp(ii) = mean(dp)*hPa2kPa;
    avg_p(ii)  = mean(p)*hPa2kPa;
    PWM(ii) = mean(pwm);
    avg_angle(ii) = mean(angle);
    angle_0(ii) = data(1,7);
       
    figure(1)
    plot(jj,avg_dp(ii),'bo')
    hold on;grid on;
    xlabel('Voltage [V]');ylabel('\Delta Pressure [kPa]')
    
    
    
end
jj = 0:1:ii-1;
figure(1)
plot(jj,avg_dp)
axis tight


pwm_range = linspace(0,4096,1000);
[p,S] = polyfit(PWM,avg_dp,3);
[p_fit,~] = polyval(p,pwm_range,S);

figure(2)
plot(PWM,avg_dp,'ro')
grid on; hold on;
% plot(PWM,avg_dp)
plot(pwm_range,p_fit)
xlabel('PWM [-]');ylabel('\Delta Pressure [kPa]')
axis tight
legend('Measured','Fit')


figure(3)
plot(jj,avg_angle,'bo')
grid on; hold on;
plot(jj,avg_angle)

