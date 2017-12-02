function est_lmk_ids = associate_msts_to_lmkID(xRob,z_list,tt_xRob,tt_xLmk_obs)

global map_lmk;
global lidar_range;
global lidar_bearing;


persistent tt_xerr;

%step 1: predict landmarks

n_lmks = size(map_lmk,1);

lmk_pred = [];
tt_lmk_pred = [];
for row = 1:n_lmks
    
    cur_lmk = map_lmk(row,:);
    
    zPred = measModel(xRob,cur_lmk(1:2)');
    
    if  (abs(angleWrapping( pi/2-zPred(2) )) < lidar_bearing ) && (zPred(1) < lidar_range)  
        
        lmk_pred = [lmk_pred; cur_lmk];
        
    end;
   

    tt_zPred = measModel(tt_xRob,cur_lmk(1:2)');
    
    if  (abs(angleWrapping( pi/2-tt_zPred(2) )) < lidar_bearing ) && (tt_zPred(1) < lidar_range)  
        
        tt_lmk_pred = [tt_lmk_pred; cur_lmk];
        
    end;



end;

tt_xerr = [tt_xerr (xRob - tt_xRob)];



% %step 2: transform msts from polar to cartesian and then local to gloabl
% step 3: associate zG_list to lmk ids using NN

nLmks_pred = size(lmk_pred,1);

nObs = size(z_list,2);


% est_lmk_ids = zeros(1,nObs);


est_lmk_ids = [];

map_id = -1;
for m =1:nObs
    
    cur_z = z_list(:,m);
    
    alpha = xRob(3)+ cur_z(2);
    
    xLmk_obs = xRob(1:2)+cur_z(1)*[cos(alpha); sin(alpha)];

    min_dist = inf;

    for iPred = 1:nLmks_pred
        
        xLmk_pred = lmk_pred(iPred,1:2)';
        
        cur_dist =  norm(xLmk_obs-xLmk_pred);
        
        if (cur_dist< min_dist)
            
            min_dist = cur_dist;
            
            map_id = lmk_pred(iPred,3);
        end;
        
    end
    
    if map_id == -1
        
        tt_xRob
        
        xRob
        
        
        tt_lmk_pred
        
        
        lmk_pred
        
        tt_xLmk_obs
        
        xLmk_obs
        
        tt_zPred
        
        zPred
        
        tt_xerr
        
        
        pause;
    end;
    
    est_lmk_ids(m) = map_id;

%      est_lmk_ids = [est_lmk_ids map_id];
    
end;

