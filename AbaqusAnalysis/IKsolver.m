function Enorm = IKsolver(x,z_mid,y_mid)

%% Get the data points

y = y_mid;
z = z_mid;

% obtain end effector y and z

% obtain rotation from the top plate



%% Get the IK output

q1 = x(1);
q2 = x(2);

L = 1;
Nmode = 1;
shape = 'cheby';

[r_opt,~,~,~] = Kinematics(x,L,Nmode,shape);  % Check output of algorith

x_opt = r_opt(:,1);                                   % optimized x 
z_opt = r_opt(:,3);



%% Define error norm to be minimized

Enorm  = ( y_mid  - x_opt ).^2
