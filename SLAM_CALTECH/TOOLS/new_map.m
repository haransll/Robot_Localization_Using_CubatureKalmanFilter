function [map, ground] = new_map(map, ground),
%-------------------------------------------------------
% University of Zaragoza
% Centro Politecnico Superior
% Robotics and Real Time Group
% Authors:  J. Neira, J. Tardos
% Date   :  7-2004
%-------------------------------------------------------
%-------------------------------------------------------
map.n = 0;
map.x = [0 0 0]';
map.P = zeros(3,3);
map.ground_id = [];
map.estimated(1).x = [0 0 0]';
map.estimated(1).P = map.P;
map.odometry(1).x = [0 0 0]';
map.odometry(1).P = map.P;

ground.trajectory(1).x = [0 0 0]';
ground.trajectory(1).P = zeros(3, 3);
