function [est_lmk_ids,obs_hyp] = probabilistic_data_association(xRob,PsqrtPred,z_list)

global map_lmk;
global lidar_range;
global lidar_bearing;
global sigma_mst_est;

%step 1: predict landmarks

n_lmks = size(map_lmk,1);

lmk_pred = [];
for row = 1:n_lmks
    
    cur_lmk = map_lmk(row,:);
    
    zPred = measModel(xRob,cur_lmk(1:2)');
    
    if  (abs(angleWrapping( pi/2-zPred(2) )) < lidar_bearing ) && (zPred(1) < lidar_range)
        
        lmk_pred = [lmk_pred; cur_lmk];
        
    end;
    
    
end;

% %step 2: transform msts from polar to cartesian and then local to gloabl
% step 3: associate zG_list to lmk ids using NN

nLmks_pred = size(lmk_pred,1);

nObs = size(z_list,2);

est_lmk_ids = [];

map_id = -1;

obs_hyp = [];

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
        
        obs_hyp = [obs_hyp false];
        

    else
        
        est_lmk_ids = [est_lmk_ids map_id];
        
        %compute mahalnobis distance
 
        xLmk = map_lmk(map_lmk(:,3)==map_id,1:2)';
        
        
        
        nx = 3;

        nPts = 2*nx;   % No. of Cubature Points
        
        CPtArray = sqrt(nPts/2)*[eye(nx) -eye(nx)];
        Xc =  repmat(xRob,1,nPts) + PsqrtPred*CPtArray;
        Xc(3,:) = angleWrapping(Xc(3,:));
        
        Zc = measModel(Xc,xLmk);
        zPred = sum(Zc,2)/nPts;      % predicted Measurement        
        zPred(2,:) = angleWrapping(zPred(2,:));
        
        Z = (Zc-repmat(zPred,1,nPts));
        Z(2,:) = angleWrapping(Z(2,:));

        [~,Sc] = qr([Z/sqrt(nPts) sigma_mst_est]',0);
        
        innov = (cur_z-zPred);
        innov(2) = angleWrapping(innov(2));

        
        y = Sc'\innov;
        d2 = (y'*y);
        
        if d2<chi2inv(0.95,2)
            
            obs_hyp = [obs_hyp true];
        else
            obs_hyp = [obs_hyp false];
        end
        
        
    end
    

end;

