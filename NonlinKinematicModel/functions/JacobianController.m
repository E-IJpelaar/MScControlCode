function u = JacobianController(J,L0,k,e,r)

%actuator 2 config space
L = L0*(1+e);
theta = k*L; % rad

% config 2 control space
if k == 0
    x_pos = 0;
    y_pos = L;
else
    x_pos = 1/k*(1-cos(theta));
    y_pos = 1/k*sin(theta);
end

y = [x_pos;y_pos;theta];

% plot end-effector location
figure(10)
plot(x_pos,y_pos,'x')
hold on;grid on;
axis([-0.1 0.1 0 0.2])
xlabel('x');ylabel('y')

% error
error = r-y;

% controller 
Kp = 1000*eye(3);
u = J'*(Kp*error);

