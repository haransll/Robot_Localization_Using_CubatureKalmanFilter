function [ok, d2, chi] = compatibility_test (map, vehicle)
%-------------------------------------------------------
% carries out the specified vehicle motion
%-------------------------------------------------------
global configuration;
global chi2;

x = map.ground.x(1, map.origin);
y = map.ground.x(2, map.origin);

g = [vehicle.ground(end).x; reshape([x; y], [], 1)];
d2 = mahalanobis( map.x - g, map.P);

chi = chi2(length(g));
ok = d2 <= chi;

if (~ok)
    for i=1:length(g),
        d2i = mahalanobis (map.x(i) - g(i), map.P(i,i));
        if d2i > chi2(1)
            disp(sprintf('i %d, x %f, g %f, P %f, d2i %f', i, map.x(i), g(i), map.P(i,i), d2i));
        end
    end
        d2i = mahalanobis(map.x(1:3) - g(1:3), map.P(1:3,1:3));
        if d2i > chi2(3)
            disp(sprintf('vehicle d2i %f', d2i));
        end
    for i=1:4:2:length(x),
        d2i = mahalanobis(map.x(i:i+1) - g(i:i+1), map.P(i:i+1,i:i+1));
        if d2i > chi2(2)
            disp(sprintf('i d2i %f', i, d2i));
        end
    end
end
