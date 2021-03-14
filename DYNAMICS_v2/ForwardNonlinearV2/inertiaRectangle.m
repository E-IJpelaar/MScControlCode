function [m_sigma,J1,J2,J3] = inertiaRectangle(m,L0,w,d,d_sigma)

% m = mass
% L = length undeformed
% w = width
% d = depth
% h = height of slice
h = d_sigma ;
rho = m/L0;                     % line density [kg/m]
m_sigma = rho;                 % [kg/m] mass of point
J1 = (rho/12)*(w^2+d^2);     % J1 [kgm] (twist) among elongation direction
J2 = (rho/12)*(w^2+h^2);     % J2 [kgm] (curvature) in plane
J3 = (rho/12)*(d^2+h^2);     % J3 [kgm] (curvature) out of plane


