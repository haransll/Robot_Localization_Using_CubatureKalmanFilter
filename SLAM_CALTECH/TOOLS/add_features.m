function map = add_features (map, observations, which)
%-------------------------------------------------------
% University of Zaragoza
% Centro Politecnico Superior
% Robotics and Real Time Group
% Authors:  J. Neira, J. Tardos
% Date   :  7-2004
%-------------------------------------------------------
%-------------------------------------------------------

twr = map.x(1:3);
Pr = map.P(1:3,1:3);
C = map.P(1:3, 4:end);

x1 = twr(1);
y1 = twr(2);
t1 = twr(3);
J2i = [cos(t1) -sin(t1)
    sin(t1)  cos(t1)];

J1 = [];
J2 = [];

if nargin < 3 % which is given
    which = 1:observations.m;
end
observations = select_observations (observations, which);

p = [observations.z(1:2:end)'; observations.z(2:2:end)'];
twp = tpcomp (twr, p);
map.x = [map.x; reshape(twp, [], 1)];

for i=1:2:length(observations.z),
    x2 = observations.z(i);
    y2 = observations.z(i+1);
    J1i = [1 0 (-x2*sin(t1) - y2*cos(t1))
        0 1  (x2*cos(t1) - y2*sin(t1))];
    J1 = [J1
        J1i];
    J2 = blkdiag(J2, J2i);
end

R = map.P(1:3,:);
R = J1 * R;

P = J1*Pr*J1' + J2*observations.R*J2';
map.P = [map.P R'
    R P];
    
map.ground_id = [map.ground_id observations.ground_id];
map.n = map.n + observations.m;
