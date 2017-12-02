function draw_hypothesis (prediction, observations, H, name, color)
%-------------------------------------------------------
% University of Zaragoza
% Centro Politecnico Superior
% Robotics and Real Time Group
% Authors:  J. Neira, J. Tardos
% Date   :  7-2004
%-------------------------------------------------------
global configuration;

if configuration.step_by_step
    
    vehicle.x = [0 0 0]';
    vehicle.P = zeros(3, 3);
    
    figure(configuration.hypothesis); clf; axis equal; hold on;
    draw_vehicle(vehicle.x, vehicle.P, 'k');
    draw_prediction (prediction, 'k');
    draw_obs(observations);
    
    % paired observations
    Es = find(H);
    [ix, iy, ind] = obs_rows(Es);
    xe = observations.z(ix);
    ye = observations.z(iy);
    
    % paired features
    Fs = H(find (H));
    [ix, iy, ind] = obs_rows(Fs);
    xf = prediction.h(ix);
    yf = prediction.h(iy);
    
    %pairings
    for p=1:length(Es),
        %h = plot([xf(p) ; xe(p)], [yf(p); ye(p)], color);
        %set(h, 'LineWidth', 1.5);
        arrow([xe(p) ye(p)], [xf(p) yf(p)], color);
    end
    
    title([name ': ' sprintf('%d ', H)]);
    %axis([0 3.5 -1.5 1]);
    pause
end