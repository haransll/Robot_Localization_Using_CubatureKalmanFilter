function show_results(map, ground, results),
%-------------------------------------------------------
% University of Zaragoza
% Centro Politecnico Superior
% Robotics and Real Time Group
% Authors:  J. Neira, J. Tardos
% Date   :  7-2004
%-------------------------------------------------------
% slam, explore data association algorithms
%-------------------------------------------------------
global chi2;

figure; hold on;

subplot(2, 2, 1); hold on;
bar(results.true.positives./results.total); axis([1 length(ground.motion) 0 1]);
title('True positives');

subplot(2, 2, 2); hold on;
bar(results.true.negatives./results.total); axis([1 length(ground.motion) 0 1]);
title('True negatives');

subplot(2, 2, 3); hold on;
bar(results.false.positives./results.total); axis([1 length(ground.motion) 0 1]);
title('False positives');

subplot(2, 2, 4); hold on;
bar(results.false.negatives./results.total); axis([1 length(ground.motion) 0 1]);
title('False negatives');

g = [ground.trajectory(:).x];
e = [map.estimated(:).x];

es = [map.estimated(:).P];

figure; hold on;
subplot(2, 2, 1); hold on;
title('Vehicle error in x (m)');
plot(g(1,:)-e(1,:), 'b-');
plot(sqrt(chi2(1)*es(1,1:3:end)), 'r-');
plot(-sqrt(chi2(1)*es(1,1:3:end)), 'r-');

subplot(2, 2, 2); hold on;
title('Vehicle error in y (m)');
plot(g(2,:)-e(2,:), 'b-');
plot(sqrt(chi2(1)*es(2,2:3:end)), 'r-');
plot(-sqrt(chi2(1)*es(2,2:3:end)), 'r-');

subplot(2, 2, 3); hold on;
title('Vehicle error in theta (deg)');
plot((normalize(g(3,:)-e(3,:)))*180/pi, 'b-');
plot(sqrt(chi2(1)*es(3,3:3:end))*180/pi, 'r-');
plot(-sqrt(chi2(1)*es(3,3:3:end))*180/pi, 'r-');

