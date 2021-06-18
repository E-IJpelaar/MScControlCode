clear all;close all;clc;
%% Observe dynamic model steps with alpha Kk and beta Ke



for ii = 12
    
    data = load(['simdata',num2str(ii),'V.mat']);
    
    texp = data.texp;
    p1 = data.p1;
    p01 = data.p01;
    p2 = data.p2;
    p02 = data.p02;
    u2 = data.u2;
    u1 = data.u1;
    expe = data.epsilon;
    expa = data.angle;
    expk = data.kappa;
    expp1 = (p1-p01)./10;
    expp2 = (p2-p02)./10;
    
    tsim = data.t;
    sime = data.e;
    simk = data.k;
    simrot = data.rot;
    simp1 = data.p1sim;
    simp2 = data.p2sim;
    
    figure(1)
    plot(tsim,sime)
    hold on; grid on;
    plot(texp,expe)
    
    
%     figure(2)
%     plot(tsim,simk)
%     hold on; grid on;
%     plot(texp,expk)
%     
%     
%     
%     figure(3)
%     plot(tsim,simrot)
%     hold on; grid on;
%     plot(texp,expa)
%     
%     figure(4)
%     plot(tsim,simp1)
%     hold on; grid on;
%     plot(texp,expp1)
    
    
    
    
end
