function draw_map (map, ground, step)
%-------------------------------------------------------
% University of Zaragoza
% Centro Politecnico Superior
% Robotics and Real Time Group
% Authors:  J. Neira, J. Tardos
% Date   :  7-2004
%-------------------------------------------------------
%-------------------------------------------------------
global configuration;
global people;

figure(configuration.map); clf; axis equal; hold on;
%axis([-2 12 -2 12]);
% draw vehicle
draw_vehicle(map.x(1:3), map.P(1:3,1:3), 'b');

% ground truth for trajectory
draw_trajectory(ground.trajectory, 'r', step);

% trajectory up to now
draw_trajectory(map.estimated, 'b');

% odometry up to now
%draw_trajectory(map.odometry, 'g');

%draw features
x = map.x(4:2:end);
y = map.x(5:2:end);
plot(x, y, ['b' '+']);

for p = 1:length(x),
    ind = 2*p+2:2*p+3;
    if configuration.ellipses
        draw_ellipse (map.x(ind), map.P(ind, ind), 'b');
    end
    if configuration.tags
        ht = text(x(p)-0, y(p)-0.05, ['F' num2str(p)]);
        set(ht, 'Color', 'b');
    end
end

% ground truth for features
[i, j, ground_id] = find(map.ground_id);

plot(ground.points(1, ground_id), ground.points(2, ground_id), 'r.');

if configuration.people
    for p=1:size(people.x,2),
    plot(people.x(:,p), people.y(:,p), 'r.');
end
end

title(sprintf('MAP at Step %d, features: %d, algorithm: %s', step, map.n, configuration.name));
if configuration.step_by_step
    pause
else
    drawnow;
end

