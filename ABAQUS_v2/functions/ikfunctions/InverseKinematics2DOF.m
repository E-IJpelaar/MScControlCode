function  [xy,q_opt]  = InverseKinematics2DOF(x_d,epsilon,Nmode,L0)
%% Initial conditions
shape = "cheby";             % poly = polynomial, cheby = chebyshev, legendre = legendre
rho = 1e-1;
%% IK parameters
q0 = zeros(2*Nmode,1);     % initial guess
it_max = 1000;             % maximum amount of iterations
it = 0;                    % set iterations to zero
alpha = 50;                % learning gain
w = kron(eye(2),diag([ones(1,Nmode)])); % each q is equally important for diag(1)
beta = diag([1,1]);      % each state q is equally important for diag(1)

%% Check initial guess
[r,R,~,~] = funcKinematics(q0,L0,Nmode,shape);      % Check for intial guess

R_end = quat2rotm(R(end,:));                % rotation matrix at sigma = L   
x = r(end,1);                              % end-effector x position
z = r(end,3);                              % end-effector z position

f_q0 = [x;z];                              % f(q) position for q0

e = x_d-f_q0;                              % error between desired position and first guess
e_norms = [0,norm(e)+0.1];                 % stack error norms for last k-1 and k to check for convergence
%% Optimizing q0
while norm(e) > epsilon        % loop until error is smaller than max error norm & maxium iterations is not exceeded

        [r,R,l,Ba] = funcKinematics(q0,L0,Nmode,shape); % call forward kinematics script
    
        x = r(end,1);                              % end-effector x position at sigma = L
        z = r(end,3);                              % end-effector z position at sigma = L
    
        f_q0 = [x;z];                        % f(q) end-efforctor position for q0 at sigma = L
        e = x_d-f_q0;                              % error between desired position and guess
    
        J = zeros(6,2*Nmode);                      % Pre-allocate "new" Jacobian after each iteration
            
            for ii = 2:length(l)                       % Determine Jacobian by integrating over L ( int*(0,sigma) Adg*Ba*Phi(sigma) dsigma)
                dh = l(ii)-l(ii-1);
                Baphi_s = shapeValue(shape,Nmode,l(ii),Ba,L0);    % Determine Ba*Phi_s for each sigma
                Adg = adjointG(R(ii,:),r(ii,:));               % Calculate Adg for each sigma
                J = J + Adg*Baphi_s*dh;                           % Add contribution of each delta sigma to total Jacobian
    
            end
    
        invAdg = adjointGInv(R(end,:),r(end,:));             % Calculate Adg^-1 of end-effector frame
        J = invAdg*J;                                        % Obtain final Jacobian
 
              
        pInvJ = (w*J.')/(J*w*J.' + rho*eye(6));              % Determine inverse Jacobian to update q0 %damped pseudo inverse    
        pInvJ = pInvJ(:,4:2:6);                              % "Sloppy" way to only use [theta,x,z] information in Jacobian
   
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

q_opt = q0;                                           % Display optimized q0
[r_opt,~,~,~] = funcKinematics(q_opt,L0,Nmode,shape);      % Check output of algorith

x_opt = r_opt(:,1);                                   % optimized x 
z_opt = r_opt(:,3);                                   % optimized z

xy = [x_opt,z_opt];



