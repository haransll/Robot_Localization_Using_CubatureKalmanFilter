function d2 = mahalanobis_dist(z,xRob,xLmk,PsqrtPred)

global sigma_mst_est;

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

innov = (z-zPred);
innov(2) = angleWrapping(innov(2));

y = Sc'\innov;
d2 = (y'*y);