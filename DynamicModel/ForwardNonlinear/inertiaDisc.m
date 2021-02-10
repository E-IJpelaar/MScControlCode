function [m_sigma,J1,J2,J3] = inertiaDisc(m,L,w,d,h,J_cor)

% m = mass
% L = length undeformed
% w = width
% d = depth
% h = height of slice
mm2m = 1e-3;
alpha = J_cor;
rho = m/L;                     % line density [kg/m]
m_sigma = rho;                 % [kg/m] mass of point
J1 = 2*(m_sigma*((32*mm2m)^2 + (16*mm2m)^2));
% J2 = 0.25*2*(m_sigma*((32*mm2m)^2 + (16*mm2m)^2));
J2 = ((w*L)/12)*(L^2+w^2);
J3 = J2; 

