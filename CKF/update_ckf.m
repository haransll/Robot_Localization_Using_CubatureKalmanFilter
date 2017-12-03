function [xUpd,PsqrtUpd] = update_ckf(z,est_lmk_id,xPred,PsqrtPred)

global map_lmk;
global sigma_mst_est;


nx = 3;
nz = 2;

nPts = 2*nx;   % No. of Cubature Points

CPtArray = sqrt(nPts/2)*[eye(nx) -eye(nx)];

xLmk = map_lmk(map_lmk(:,3)==est_lmk_id,1:2)';


Xc =  repmat(xPred,1,nPts) + PsqrtPred*CPtArray;
Xc(3,:) = angleWrapping(Xc(3,:));

Zc = measModel(Xc,xLmk);

zPred = sum(Zc,2)/nPts;      % predicted Measurement

zPred(2,:) = angleWrapping(zPred(2,:));

X = (Xc-repmat(xPred,1,nPts));
X(3,:) = angleWrapping(X(3,:));

Z = (Zc-repmat(zPred,1,nPts));
Z(2,:) = angleWrapping(Z(2,:));

[~,S] = qr([Z/sqrt(nPts) sigma_mst_est; X/sqrt(nPts) zeros(nx,nz)]',0);

S = S';

A = S(1:nz,1:nz);   % Square-root Innovations Covariance

B = S(nz+1:end,1:nz);

C = S(nz+1:end,nz+1:end);

G = B/A;          % Cubature Kalman Gain


inn = (z-zPred);
inn(2) = angleWrapping(inn(2)); 

xUpd = xPred + G*inn;

PsqrtUpd = C;


