function [xi,dxi,ddxi] = strainField(l,q,dq,ddq,Ba)

phi = eye(length(q));  % one shape function

% phi = [1,l,0,0;
%        0,0,1,l];
   
xi = Ba*phi*q;
dxi = Ba*phi*dq;
ddxi = Ba*phi*ddq;

 



