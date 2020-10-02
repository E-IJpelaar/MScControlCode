function g_acc = odefnc(l,g)

% Phi = eye(6);
qa = [1;0.5;0.25]
q = [qa;qa;qa;qa;qa;qa];
% q   = ones(6*3,1);
O3 = zeros(1,3);

Phi = [chebyshevT([0, 1, 2], l),O3,O3,O3,O3,O3;
       O3,chebyshevT([0, 1, 2], l),O3,O3,O3,O3;
       O3,O3,chebyshevT([0, 1, 2], l),O3,O3,O3;
       O3,O3,O3,chebyshevT([0, 1, 2], l),O3,O3;
       O3,O3,O3,O3,chebyshevT([0, 1, 2], l),O3;
       O3,O3,O3,O3,O3,chebyshevT([0, 1, 2], l)];
      

eta = Phi*q;

g = reshape(g,[4,4]);

eta_hat = [ 0     , -eta(1), eta(2), eta(4);
            eta(1),  0     , eta(3), eta(5);
           -eta(4), -eta(3), 0     , eta(6);
            0     ,  0     , 0     , 0    ];

g_acc = g*eta_hat;
g_acc = reshape(g_acc,[],1);
