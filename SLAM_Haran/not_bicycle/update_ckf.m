function [xUpd,PUpd] = update_ckf(z_list,est_lmk_ids,xPred,PPred)

global map_lmk;
global sigma_mst_est;


nx = 3;
nPts = 2*nx;   % No. of Cubature Points

CPtArray = sqrt(nPts/2)*[eye(nx) -eye(nx)];

%%%========================================================================
%%% find the square-root of Pkk1 using Singular Value Decomposition (SVD)
%%%========================================================================


for i=1:size(z_list,2)
    
    PPred = 0.5*(PPred+PPred'); % make Pkk1 to be symmetric (Property of a covariance matrix !)

    Psqt_prd = chol(PPred,'lower'); 

    Xc =  repmat(xPred,1,nPts) + Psqt_prd*CPtArray;
    
    Zc = [];
    
    xLmk = map_lmk(map_lmk(:,3)==est_lmk_ids(i),1:2)';
    
    for j=1:nPts
        
        Zc = [Zc measModel(Xc(:,j),xLmk)];
        
    end;
        
    zPred = sum(Zc,2)/nPts;      % predicted Measurement
    
    zPred = angleWrapping(zPred);
    

    X = (Xc-repmat(xPred,1,nPts))/sqrt(nPts);

    Z = (Zc-repmat(zPred,1,nPts))/sqrt(nPts);  

    Pzz = Z*Z'+ sigma_mst_est.^2; % Innovations Covariance

    Pxz = X*Z'; % cross-covariance

    G = Pxz*pinv(Pzz);         % Cubature Kalman Gain  


    innov = z_list(:,i) - zPred;
    innov(2) = angleWrapping(innov(2));

    xUpd = xPred + G*innov;  

    PUpd = PPred - G*Pzz*G';

    xPred = xUpd;  

    PPred = PUpd;


end;

