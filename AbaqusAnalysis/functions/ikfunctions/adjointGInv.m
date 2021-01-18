function [AdgInv] = adjointGInv(R,r)

r_hat = [ 0   ,-r(3), r(2);
          r(3), 0   ,-r(1);
         -r(2), r(1), 0  ];  % same as code Brandon Liegroup.h. However, is still correct elongation for me is E3 in z, and for you E3 in x
     
R = quat2rotm(R(:).');
Rr = R*r_hat';     % mine gives 0,1,0;-1,0,0;0,0,0 your gives 0,0,0;0,0,1;0,-1,0
O = zeros(3,3);

AdgInv = [R, O;
          Rr,R];

