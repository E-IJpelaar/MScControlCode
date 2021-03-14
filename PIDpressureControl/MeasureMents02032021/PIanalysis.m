% clear all;close all;clc;


        data = load('PIcontrol.txt');
        % analyse raw data
        t = data(:,1);
        dp = data(:,3)*0.1;
        u = data(:,5);
        v = u./341.3333;
        
        figure(1)
        plot(t,dp)
        hold on;grid on;
        
        figure(2)
        plot(t,u)