clear all;close all;clc;

p_q1 = linspace(-1.1,1.1,100);
p_q2 = linspace(-0.6,0.6,100);

beta = [1.1579,1.1423,0.0003]*1.0e+03 ;
for ii = 1:length(p_q1)
    for jj = 1:length(p_q2)
        F_adapted(ii,jj) = (1  + 0.8788*abs(p_q1(ii))).*((beta(1) + beta(2)*((tanh(beta(3)*p_q2(jj)))^2 -1))*p_q2(jj));
        Ke_adapted(ii,jj) = (1  + 0.8788.*abs(p_q1(ii)))*((beta(1) + beta(2)*((tanh(beta(3).*p_q2(jj)))^2 -1)));
        F_or(ii,jj) = ((beta(1) + beta(2)*((tanh(beta(3)*p_q2(jj)))^2 -1))*p_q2(jj));
        Ke_or(ii,jj) = ((beta(1) + beta(2)*((tanh(beta(3).*p_q2(jj)))^2 -1)));
    end
end

% figure(1)
% surf(p_q1,p_q2,F_adapted)
% xlabel('q1');ylabel('q2');zlabel('F')
% figure(2)
% surf(p_q1,p_q2,F_or)
% xlabel('q1');ylabel('q2');zlabel('F')

% figure(3)
% surf(p_q1,p_q2,Ke_adapted)
% xlabel('q1');ylabel('q2');zlabel('K_\epsilon')

figure(4)
surf(p_q1,p_q2,Ke_or)
xlabel('q1');ylabel('q2');zlabel('K_\epsilon')
