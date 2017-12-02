function draw_compatibility (prediction, observations, compatibility)
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
    
    figure(configuration.hypothesis); clf; axis equal; hold on;
    
    vehicle.x = [0 0 0]';
    vehicle.P = zeros(3, 3);
    
    %draw vehicle
    draw_vehicle(vehicle.x, vehicle.P, 'r');
    
    %draw prediction
    draw_prediction(prediction);

    %draw observations
    draw_obs(observations);

    % draw compatibility
    for i=1:observations.m,
        xo = observations.z(2*i - 1);
        yo = observations.z(2*i);
        for j=1:prediction.n,
            if compatibility.ic(i,j)
                xf = prediction.h(2*j-1);
                yf = prediction.h(2*j);
                %h = plot([xo xf], [yo yf], 'b');
                %set(h, 'LineWidth', 1.5);
                arrow([xo yo], [xf yf], 'b');
            end
        end
    end
    
    title('Individually compatible pairings');
    pause  
    
end