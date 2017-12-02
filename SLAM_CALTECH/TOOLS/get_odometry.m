function odometry = get_odometry (motion)
%-------------------------------------------------------
% University of Zaragoza
% Centro Politecnico Superior
% Robotics and Real Time Group
% Authors:  J. Neira, J. Tardos
% Date   :  7-2004
%-------------------------------------------------------
%-------------------------------------------------------
global configuration;

if configuration.odometry
    if configuration.noise
        odometry.x = motion.x + gaussian_noise(motion.P);
    else
        odometry.x = motion.x;
    end
    odometry.P = motion.P;
else
    odometry.x = [0 0 0]';
    odometry.P = diag([0.25 0.1 5*pi/180].^2);
end
