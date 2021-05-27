load calib_data.mat

A = [Rc_3(:,1),Rc_3(:,2),Tc_3];
A(3,:) = [0,0,1]
invA = inv(A);

x_p = 150;
y_p = 220;

s = (invA(3,3) + invA(3,1)*x_p + invA(3,2)*y_p);

r = s*inv(A)*[x_p;y_p;1]

