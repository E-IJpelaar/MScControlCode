clear all;close all;clc;
%% Desired end effector position, initilization, max error

yd = [0;0.1;1.2];            % [theta,x,z]
epsilon = 0.001;           % max error norm
q0 = [0;0;0;0];        % q is set Rm m ==4;
[r,R,~,~,~,~] = Kinematics(q0);  % first gues

Rend = quat2rotm(R(end,:));        % end quaternion to rotation matrix
theta = acos((trace(Rend)-1)/2);   % calculate rotation

fq = [acos((trace(quat2rotm(R(end,:)))-1)/2);r(end,1);r(end,3)];   % actual position to define error


e = yd-fq;   % error yd (desired position), fq(actual position)

while norm(e) > epsilon   % while error > max error
  
    [r,R,l,Ba,shape,Nmode] = Kinematics(q0);

    fq = [acos((trace(quat2rotm(R(end,:)))-1)/2);r(end,1);r(end,3)];   % actual position to define error
    e = yd-fq; 

J = 0;
for ii = 1:length(l)
    Baphi_s = shapeValue(shape,Nmode,l(ii),Ba);
    Adg = adjointG(R(ii,:),r(ii,:));     
    J = J + Adg*Baphi_s;
    
end
   invAdg = adjointGInv(R(end,:),r(end,:));
   J = invAdg*J;  
   pInvJ = pinv(J);
   
   pInvJ = pInvJ(:,2:2:6);
   q0 = q0 + pInvJ*e;
   
   
end



   
        



    
% intJ =  intJ + Adg*Ba*


% while g < epsilon
%   
%     
%     
%     
%    % J = Adg^-1 int0-sigma Adg Ba phi(sigma) dsigma
%     % also obtain J from te Kinematics function see. Brandon model.dyn
%     fq = Kinematic(q0);
%     g  = yd-fq;
%     dq = ((1)^-1)*g
%     q0 = q0+dq;
%     
%     
% end

% skew symmetric matrix needed
% Y = [4;9;3];
% Y_s = [0,-Y(3),Y(2);
%        Y(3),0,-Y(1);
%        -Y(2),Y(1),0]
% V = [1;0.5;4]
% 
% YV = Y_s*V
% YxV = cross(Y,V)
% 

      



