clear all;close all;clc;

load 'elongationdatavarNmode_20201118T102655.mat'

[n,m] = size(data);
Nmodes = 1:1:m;

sumq1 = zeros(1,length(Nmodes));
sumq2 = zeros(1,length(Nmodes));

for ii = 1:length(Nmodes)
Nmode = Nmodes(ii);

p = data(1,ii);
q1= data(2:2+(Nmode-1),ii);
q2= data(2+Nmode:1+2*Nmode,ii);
E = data(end,ii);

sumq1(1,ii) = sum(q1);
sumq2(1,ii) = sum(q2);

% figure(1)
% plot(Nmode,q1,'x')
% hold on; grid on;
% plot(Nmode,sumq1(1,ii),'o')
% xlabel('Nmode');ylabel('q_1 \kappa')


figure(2)
plot(Nmode,q2,'x')
hold on; grid on;
plot(Nmode,sumq2(1,ii),'o')


end

xlabel('Nmode');ylabel('q_2 \epsilon')



