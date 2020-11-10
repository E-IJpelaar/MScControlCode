function [xi_hat] = strainField(~,~,q,Ba)


xi0 = [0;0;0;0;0;1];


phi = [1,0;
       0,1];

xi = Ba*(phi*q) + xi0;

xi_hat =  [  0       , -xi(3), xi(2) , xi(4);               
            xi(3) ,  0      , -xi(1) , xi(5);
           -xi(2) ,  xi(1)  ,  0     , xi(6);
            0       ,0      ,  0     , 0   ];