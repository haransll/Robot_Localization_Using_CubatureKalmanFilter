function draw_ground (ground),
%-------------------------------------------------------
% University of Zaragoza
% Centro Politecnico Superior
% Robotics and Real Time Group
% Authors:  J. Neira, J. Tardos
% Date   :  7-2004
%-------------------------------------------------------
%-------------------------------------------------------
global configuration;

figure(configuration.ground); clf; axis equal; hold on;
plot(ground.points(1,:), ground.points(2,:), 'r.');

trajectory(1).x = [0 0 0]';
trajectory(1).P = zeros(3, 3);
for step = 1 : length(ground.motion),
    [trajectory(step+1).x, trajectory(step+1).P] = ...
        ucomp(trajectory(step).x, trajectory(step).P, ...
        ground.motion(step).x, zeros(3,3));
end

draw_trajectory (trajectory, 'r');
title(sprintf('GROUND TRUTH, features: %d', ground.n));
