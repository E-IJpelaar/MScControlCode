function dgdl = forwardKinematics(l,g,q,Ba,shape,Nmode)
%% Convert g
g_rot = quat2rotm(g(1:4)');              % convert quaternion to rotation matrix (transpose for "quat2rot" syntax)

%% Calculate strain field
[xi] = strainField(l,q,Ba,shape,Nmode);  % get strain field

xi_rot = [  0     , -xi(3),  xi(2);               
            xi(3) ,  0    , -xi(1); 
           -xi(2) ,  xi(1),  0   ];      % rotation matrix with curvatures
             
xi_trans = xi(4:6);                      % strain part

%% g'=g*xi
dgdl_rot = rotm2quat(g_rot*xi_rot);      % curvature in quaternion expression
dgdl_trans = g_rot*xi_trans;             % translation

dgdl = [dgdl_rot';dgdl_trans];           % delta g/ delta sigma (transpose for "quat2rot" syntax)
