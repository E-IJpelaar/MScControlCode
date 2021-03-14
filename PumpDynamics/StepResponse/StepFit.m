clear all;close all;clc;

sample_delay = 3;

data2 = load('2V1.txt');
t2 = data2(1:end-sample_delay,1);
p2 = data2(sample_delay+1:end,3)*0.1;
u2 = data2(sample_delay+1:end,5);

data3 = load('3V1.txt');
t3 = data3(1:end-sample_delay,1);
p3 = data3(sample_delay+1:end,3)*0.1;
u3 = data3(sample_delay+1:end,5);

data4 = load('4V1.txt');
t4 = data4(1:end-sample_delay,1);
p4 = data4(sample_delay+1:end,3)*0.1;
u4 = data4(sample_delay+1:end,5);

data5 = load('5V1.txt');
t5 = data5(1:end-sample_delay,1);
p5 = data5(sample_delay+1:end,3)*0.1;
u5 = data5(sample_delay+1:end,5);


data6 = load('6V1.txt');
t6 = data6(1:end-sample_delay,1);
p6 = data6(sample_delay+1:end,3)*0.1;
u6 = data6(sample_delay+1:end,5);


figure(1)
plot(t2,p2)
hold on;grid on;
plot(t3,p3)
plot(t4,p4)
plot(t5,p5)
plot(t6,p6)
xlabel('Time [s]');ylabel('\Delta P[kPa]')
legend('2V','3V','4V','5V','6V')



H2 = tf(tf2.Numerator,tf2.Denominator);
H3 = tf(tf3.Numerator,tf3.Denominator);
H4 = tf(tf4.Numerator,tf4.Denominator);
H5 = tf(tf5.Numerator,tf5.Denominator);
H6 = tf(tf6.Numerator,tf6.Denominator);

