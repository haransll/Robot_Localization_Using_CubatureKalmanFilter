function ground = move_vehicle (ground, motion)
%-------------------------------------------------------
% University of Zaragoza
% Centro Politecnico Superior
% Robotics and Real Time Group
% Authors:  J. Neira, J. Tardos
% Date   :  7-2004
%-------------------------------------------------------
%-------------------------------------------------------

ground.trajectory(end+1).x = tcomp(ground.trajectory(end).x, motion.x);
ground.trajectory(end).P = zeros(3, 3);
