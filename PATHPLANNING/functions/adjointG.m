function [Adg] = adjointG(Q,r)

r_hat = [ 0   ,-r(3), r(2);
          r(3), 0   ,-r(1);
         -r(2), r(1), 0  ];
R = quat2rotm(Q);
rR = r_hat*R;
O = zeros(3,3);

Adg = [R, O;
       rR,R];

