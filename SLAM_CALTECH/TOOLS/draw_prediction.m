function draw_prediction (prediction, which)
%-------------------------------------------------------
% University of Zaragoza
% Centro Politecnico Superior
% Robotics and Real Time Group
% Authors:  J. Neira, J. Tardos
% Date   :  7-2004
%-------------------------------------------------------
%-------------------------------------------------------
global configuration;

%draw features
if nargin < 3
   which = 1:prediction.n;
end
[ix, iy, rows] = obs_rows(which);
x = prediction.h(ix);
y = prediction.h(iy);

plot(x, y, ['b' '+']);

for p = which,
    ind = 2*p-1:2*p;
    if configuration.ellipses
        draw_ellipse (prediction.h(ind), prediction.HPH(ind, ind), 'b');
    end
    if configuration.tags
        ht = text(x(p)-0, y(p)-0.05, ['F' num2str(p)]);
        set(ht, 'Color', 'b');
    end
end

[i, j, ground_id] = find(prediction.ground_id);
plot(prediction.ground(1, ground_id), prediction.ground(2, ground_id),'r.');
