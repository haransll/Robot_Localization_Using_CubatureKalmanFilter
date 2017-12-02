function draw_obs (observations)
%-------------------------------------------------------
% University of Zaragoza
% Centro Politecnico Superior
% Robotics and Real Time Group
% Authors:  J. Neira, J. Tardos
% Date   :  7-2004
%-------------------------------------------------------
%-------------------------------------------------------
global configuration;

%draw observations
x = observations.z(1:2:end);
y = observations.z(2:2:end);
plot(x, y, ['g' '+']);

for p = 1:length(x),
    if configuration.ellipses
        draw_ellipse (observations.z(2*p-1:2*p), observations.R(2*p-1:2*p, 2*p-1:2*p), 'g');
    end
    if configuration.tags
        ht = text(x(p)-0, y(p)+0.05, ['O' num2str(p)]);
        set(ht, 'Color', 'g');
    end  
end
