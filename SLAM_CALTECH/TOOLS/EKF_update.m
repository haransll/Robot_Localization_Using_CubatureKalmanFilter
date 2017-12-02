function map2 = EKF_update (map, prediction, observations, H)
%-------------------------------------------------------
% University of Zaragoza
% Centro Politecnico Superior
% Robotics and Real Time Group
% Authors:  J. Neira, J. Tardos
% Date   :  7-2004
%-------------------------------------------------------
%-------------------------------------------------------

map2 = map;

[d2k, Hk, Ck, hk, zk, Rk] = joint_mahalanobis2 (prediction, observations, H);

Kk = map.P * Hk' * inv(Hk * map.P * Hk' + Rk);

xk = map.x + Kk * (zk - hk);
Pk = (eye(size(map.P)) - Kk * Hk) * map.P;

map2.x = xk;
map2.P = Pk;
map2.estimated(end+1).x = xk(1:3);
map2.estimated(end).P = Pk(1:3,1:3);