function map = EKF_prediction (map, motion)
%-------------------------------------------------------
% University of Zaragoza
% Centro Politecnico Superior
% Robotics and Real Time Group
% Authors:  J. Neira, J. Tardos
% Date   :  7-2004
%-------------------------------------------------------
%-------------------------------------------------------
global configuration;

twr = map.x(1:3);
trn = motion.x;

j1 = jacobian1(twr, trn);
j2 = jacobian2(twr, trn);

J1 = sparse(blkdiag(j1, eye(2 * map.n)));
J2 = sparse([j2;  zeros(2 * map.n, 3)]);
map.x(1:3) = tcomp (twr, trn);
map.P = J1 * map.P * J1' + J2 * motion.P * J2';

tab = map.odometry(end).x;
Pab = map.odometry(end).P;
tbc = trn;
Pbc = motion.P;
[tac, Pac] = ucomp(tab, Pab, tbc, Pbc);
map.odometry(end+1).x = tac;
map.odometry(end).P = Pac;

