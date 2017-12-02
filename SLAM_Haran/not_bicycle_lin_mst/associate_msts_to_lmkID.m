function est_lmk_ids = associate_msts_to_lmkID(xRob,z_list)

global map_lmk;
global lidar_range;
global lidar_bearing;

%step 1: predict landmarks

n_lmks = size(map_lmk,1);

lmk_pred = [];

for row = 1:n_lmks
    
    xLmk = [map_lmk(row,1); map_lmk(row,2)];
    
    zPred = measModel(xRob,xLmk);
    
    if ((zPred(1) < lidar_range) && (abs(angleWrapping(pi/2-zPred(2))) < lidar_bearing))
        
        lmk_pred = [lmk_pred; map_lmk(row,:)];
        
    end;
   
end;



% %step 2: transform msts from polar to cartesian and then local to gloabl




nObs = size(z_list,2);

zG_list = [];

for m =1:nObs
    
    z_m = z_list(:,m);
    
    alpha = xRob(3)+ z_m(2);

    zG_list = [zG_list  [xRob(1)+z_m(1)*cos(alpha); xRob(2)+z_m(1)*sin(alpha)] ];  %transformed (to cartesian cordinate) in the gloabl FW
    
end


% step 3: associate zG_list to lmk ids using NN
% est_lmk_ids = zeros(1,nObs);
est_lmk_ids = [];
for iobs = 1:nObs
    
    min_dist = 1e6;
    
    map_id = [];
    
    cur_zG = zG_list(:,iobs);
    
    nLmks_pred = size(lmk_pred,1);
    
    for ilmk = 1:nLmks_pred
        
        xlmk = lmk_pred(ilmk,1:2)';
        
        cur_dist =  norm(cur_zG-xlmk);
        
        if (cur_dist< min_dist)
            
            min_dist = cur_dist;
            
            map_id = lmk_pred(ilmk,3);
        end;
        
    end
    
%     est_lmk_ids(1,iobs) = map_id;

    est_lmk_ids = [est_lmk_ids map_id];
    
end;

