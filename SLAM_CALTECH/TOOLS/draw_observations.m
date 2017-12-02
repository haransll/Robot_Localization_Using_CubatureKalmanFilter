function draw_observations (observations, ground, step, which)
%-------------------------------------------------------
% University of Zaragoza
% Centro Politecnico Superior
% Robotics and Real Time Group
% Authors:  J. Neira, J. Tardos
% Date   :  7-2004
%-------------------------------------------------------
%-------------------------------------------------------
global configuration;

if configuration.step_by_step
    figure(configuration.observations); clf; axis equal; hold on;
    
    vehicle.x = [0 0 0]';
    vehicle.P = zeros(3, 3);
    
    %draw vehicle
    draw_vehicle(vehicle.x, vehicle.P, 'b');
    
    if nargin == 4 % which is used
        [ix, iy, ind] = obs_rows(which);
        observations.z = observations.z(ind);
        observations.R = observations.R(ind,ind);
        observations.m = length(which);
    end
    
    %draw observations
    draw_obs (observations);
    
    for p = 1:observations.m,
        if configuration.ellipses
            draw_ellipse (observations.z(2*p-1:2*p), observations.R(2*p-1:2*p, 2*p-1:2*p), 'g');
        end
        if configuration.tags
            ht = text(observations.z(2*p-1)-0, observations.z(2*p)+0.05, ['O' num2str(p)]);
            set(ht, 'Color', 'g');
        end  
    end
    
    plot(observations.ground(1, :), observations.ground(2, :),'r.');
    
    title(sprintf('OBSERVATIONS at step %d: %d', step, observations.m));
    pause
end
