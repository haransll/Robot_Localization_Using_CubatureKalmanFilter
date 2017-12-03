function [xPred,PsqrtPred] = predict_ckf(xUpd,PsqrtUpd,u)

global sigma_u_est;

nx = 3;

nAug = 6; % state vector dimension

nPts = 2*nAug;   % No. of Cubature Points

CPtArray = sqrt(nPts/2)*[eye(nAug) -eye(nAug)];


PAug_sqrt = zeros(nAug,nAug);
PAug_sqrt(1:nx,1:nx) = PsqrtUpd;
PAug_sqrt(nx+1:end,nx+1:end) = sigma_u_est;

xAug = [xUpd;u];

Xc = repmat(xAug,1,nPts) + PAug_sqrt*CPtArray;

Xc(3,:) = angleWrapping(Xc(3,:));
XcPred =  processModel(Xc(1:nx,:),Xc(nx+1:end,:));


xPred = sum(XcPred,2)/nPts; 
xPred(3) = angleWrapping(xPred(3));

X = (XcPred-repmat(xPred,1,nPts));
X(3,:) = angleWrapping(X(3,:));

[~,S] = qr(X'/sqrt(nPts),0);

PsqrtPred = S';



