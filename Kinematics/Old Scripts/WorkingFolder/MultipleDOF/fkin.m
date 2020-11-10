function dydl = forwardKinematics(l,g,q,Ba)

xi0 = [0;0;0;0;0;1];


[xi_hat] = strainField(l,g,q,Ba)

phi = [1,0;
       0,1];

xi = Ba*(phi*q) + xi0;

g = reshape(g,4,4);

xi_hat =  [  0       , -xi(3), xi(2) , xi(4);               
            xi(3) ,  0      , -xi(1) , xi(5);
           -xi(2) ,  xi(1)  ,  0     , xi(6);
            0       ,0      ,  0     , 0    ];

dydl = g*xi_hat;
dydl = reshape(dydl,16,1);