function [F,K] = evalForce(alpha,q)

K = (alpha(1) + alpha(2).*((tanh(alpha(3).*q)).^2 -1)); 
F = K.*q;
