clear all;close all;clc;tic;
%% Initial conditions
Nmode = 1;                  % # shape functions to approximate strain/curvature
shape = "cheby";             % poly = polynomial, cheby = chebyshev, legendre = legendre
L     = 1;                  % undeformed length of actuator
rho = 1e-1;
%% IK parameters
% x_d = [1.1;0.6;0.97];      % desired end-effector position (theta,x,z) 
x_d = [1.0764;    0.6178;    0.9839];
epsilon = 0.01;            % max error norm
q0 = zeros(2*Nmode,1);     % initial guess
it_max = 5000;             % maximum amount of iterations
it = 0;                    % set iterations to zero
alpha = 25;                % learning gain
w = kron(eye(2),diag([ones(1,Nmode)])); % each q is equally important for diag(1)
beta = diag([1,1,1]);      % each state q is equally important for diag(1)
%% Error
if length(q0) ~= 2*Nmode                    % throw error when q is not of satisfactory length
    error('q0 should be of length 2*Nmode') % 2 strains (E3 and K2) are taken into account, hence 2
end

%% Check initial guess
[r,R,~,~] = Kinematics(q0,L,Nmode,shape);      % Check for intial guess

R_end = quat2rotm(R(end,:));                % rotation matrix at sigma = L
theta = atan2(R_end(1,3),R_end(1,1));       % rotation at sigma = L      
x = r(end,1);                              % end-effector x position
z = r(end,3);                              % end-effector z position

f_q0 = [theta;x;z];                        % f(q) position for q0
e = x_d-f_q0;                              % error between desired position and first guess
e_norms = [0,norm(e)+0.1];
%% Optimizing q0
while norm(e) > epsilon        % loop until error is smaller than max error norm & maxium iterations is not exceeded

        [r,R,l,Ba] = Kinematics(q0,L,Nmode,shape); % call forward kinematics script
    
        R_end = quat2rotm(R(end,:));               % rotation matrix at sigma = L
        theta = atan2(R_end(1,3),R_end(1,1));
        x = r(end,1);                              % end-effector x position at sigma = L
        z = r(end,3);                              % end-effector z position at sigma = L
    
        f_q0 = [theta;x;z];                        % f(q) end-efforctor position for q0 at sigma = L
        e = x_d-f_q0;                              % error between desired position and guess
    
        J = zeros(6,2*Nmode);                      % Pre-allocate "new" Jacobian after each iteration
            
            for ii = 1:length(l)                       % Determine Jacobian by integrating over L ( int*(0,sigma) Adg*Ba*Phi(sigma) dsigma)
        
                Baphi_s = shapeValue(shape,Nmode,l(ii),Ba);    % Determine Ba*Phi_s for each sigma
                Adg = adjointG(R(ii,:),r(ii,:));               % Calculate Adg for each sigma
                J = J + Adg*Baphi_s;                           % Add contribution of each delta sigma to total Jacobian
    
            end
    
        invAdg = adjointGInv(R(end,:),r(end,:));             % Calculate Adg^-1 of end-effector frame
        J = invAdg*J;                                        % Obtain final Jacobian
 
              
        pInvJ = (w*J.')/(J*w*J.' + rho*eye(6));              % Determine inverse Jacobian to update q0 %damped pseudo inverse    

        
        pInvJ = pInvJ(:,2:2:6);                              % "Sloppy" way to only use [theta,x,z] information in Jacobian
   
        q0 = q0 +  alpha*pInvJ*beta*e;                       % update rule q
        it = it+1;                                           % keep track of iterations
 
        if it >= it_max
            warning('Optimization has stopped, maximum iterations reached')
            break;
        end
        norme = norm(e);
        e_norms = [e_norms(2), norme];
        
        if abs(e_norms(1)-e_norms(2)) <= 1e-8
            warning('Optimization has stopped, error does not further converge. Increasing N_mode might help')
            break;
        end

        disp(['Number of iterations ', num2str(it)])
        disp(['Error norm ', num2str(norme)])
end
%% Compare desired end-effector position with achieved
toc

q_opt = q0;                                           % Display optimized q0
[r_opt,R_opt,~,~] = Kinematics(q_opt,L,Nmode,shape);  % Check output of algorith

R_end_opt = quat2rotm(R_opt(end,:));                  % rotation matrix at sigma = L
theta_opt = atan2(R_end_opt(1,3),R_end_opt(1,1));

x_opt = r_opt(:,1);                                   % optimized x 
z_opt = r_opt(:,3);                                   % optimized z
arctuator_length = arclength(x_opt,z_opt,'s');        % actuator length

figure(1)                                             % z desired and z optimum are inverted to show robot configuration
plot(x_d(2),x_d(3),'x','MarkerSize',12)
hold on; grid on; box on;
plot(x_opt,z_opt,'LineWidth',2)
xlabel('y [-]','FontSize',12);ylabel('z [-]','FontSize',12)
axis([0 0.6  0 1.2])
legend('Desired coordinate','IK solution','FontSize',12)
axis([0 1 0 1])
disp(['Error in x = ', num2str(x_d(2)- x_opt(end)) ,' [m]']);
disp(['Error in z = ', num2str(x_d(3)- z_opt(end)) ,' [rad]']);
disp(['Error in theta = ', num2str(x_d(1)- theta_opt) ,' [rad]']);
disp(['Amount of iterations = ', num2str(it)])
disp(['Max achievable Error norm = ', num2str(norme)])




