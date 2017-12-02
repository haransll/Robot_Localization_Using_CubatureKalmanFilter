function wP = update_weights(z_list,xP)

global nP;
global map_lmk;

wP = ones(1,nP)/nP;

for i=1:nP
    
    
    est_lmk_ids = associate_msts_to_lmkID(xP(:,i),z_list);
    
    for m = 1:size(z_list,2)
        
        if ~isempty(est_lmk_ids)
        
        
            xLmk = [map_lmk(est_lmk_ids(m),1); map_lmk(est_lmk_ids(m),2)];

            zPred = measModel(xP(:,i),xLmk);

            innov = (z_list(:,m) - zPred);

            wP(i) = wP(i)*gauss_lh(innov);
        
        end;
       
    end
   
    
end;  % end of weights


% Normalize weight vector
wP = wP./sum(wP);



function lh = gauss_lh(v)

global sigma_mst_est;


sig1 = sigma_mst_est(1,1);
sig2 = sigma_mst_est(2,2);

e1 = -0.5*v(1)^2/sig1^2 ;

e2 = -0.5*v(2)^2/sig2^2;

lh = exp(e1+e2)/(2*pi*sig1*sig2);






