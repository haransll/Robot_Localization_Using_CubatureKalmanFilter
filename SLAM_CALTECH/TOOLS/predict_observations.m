%-------------------------------------------------------
function prediction = predict_observations (map, ground)
%-------------------------------------------------------
global chi2;

xr = map.x(1);
yr = map.x(2);
pr = map.x(3);

J2 = [cos(pr) sin(pr)
     -sin(pr) cos(pr)];

prediction.n = map.n; 
prediction.ground_id = map.ground_id;
prediction.h = zeros (2 * map.n, 1);
prediction.h = [];
prediction.HPH = zeros (2 * map.n, 2 * map.n + 3);
prediction.HPH = [];
prediction.H = zeros (2 * map.n, 3);
prediction.H = [];

for j=1:map.n,
    
    xj = map.x(3 + 2 * j - 1);
    yj = map.x(3 + 2 * j);
    
    hj = [(xj - xr) * cos(pr) + (yj - yr) * sin(pr)
          (xr - xj) * sin(pr) + (yj - yr) * cos(pr)];
           
    prediction.h = [prediction.h; 
        hj];
    
    Hrj = [-cos(pr), -sin(pr), -(xj - xr) * sin(pr) + (yj - yr) * cos(pr)
            sin(pr), -cos(pr),  (xr - xj) * cos(pr) - (yj - yr) * sin(pr)];
             
    Hj = sparse([Hrj zeros(2, 2 * (j-1)) J2 zeros(2, 2 * (map.n - j))]);
    
    prediction.H = [prediction.H; 
        Hj];
    
end

prediction.HPH =  prediction.H * map.P * prediction.H';

twr = ground.trajectory(end).x;
trw = tinv(twr);
prediction.ground = tpcomp(trw, ground.points);