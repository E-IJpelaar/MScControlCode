function [p_sim,rot_sim] = steadystateRot(data)

data_sim = sortrows(data.',1).'; % sort data from low to high pressure
p_sim = data_sim(1,:);           % pressure kPa of one bellow (curvature analysis is considered)
theta_sim = data_sim(5,:);         % fifth index contains rotations (rad)
rot_sim = zeros(1,length(theta_sim));% prealocate
for ii = 1:length(p_sim)
    rot = theta_sim(ii);
    if rot <= 0.5*pi                     % [deg] rotation is already correct
        rot_sim(ii)= rad2deg(rot);
    else
        rot_sim(ii)= rad2deg(pi-rot);  % [deg] get relative rotation correct
    end
end    