function [z,lmk_ids] = sense(xRob)

global sigma_mst;
global lidar_bearing;
global lidar_range;
global map_lmk;


%init
z = [];
lmk_ids = [];  % if no lmk is detected

nLmks = size(map_lmk,1);

cnt = 0;

for i=1:nLmks
    
    xLmk = map_lmk(i,1:2);
    xLmk = xLmk';
    
    zTemp = measModel(xRob,xLmk)+sigma_mst*randn(2,1);
    
    zTemp(2) = angleWrapping(zTemp(2));
    
    
    if ( (abs(angleWrapping((pi/2 - zTemp(2))))<lidar_bearing) && (zTemp(1)<lidar_range))
        
        z = [z zTemp];
        
        lmk_ids = [lmk_ids i];
        
        cnt = cnt+1;
        
        fprintf('Landmark Found !! ID:%d \n',i);
        
%          break;
        
    end
    
    fprintf('total  landmarks found :%d \n\n',cnt);

    
end;








