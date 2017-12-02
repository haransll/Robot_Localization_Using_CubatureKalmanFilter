function [xPred,PPred] = predict_ckf(xUpd,PUpd,u)

global sigma_u_est;

nx = 3;

nAug = 5; % state vector dimension

nPts = 2*nAug;   % No. of Cubature Points

CPtArray = sqrt(nPts/2)*[eye(nAug) -eye(nAug)];

PAug = zeros(nAug,nAug);

PAug(1:nx,1:nx) = PUpd;
PAug(nx+1:end,nx+1:end) = sigma_u_est.^2;


PAug = 0.5*(PAug+PAug'); % make Pkk1 to be symmetric (Property of a covariance matrix !)

PAug_sqrt = chol(PAug,'lower');

xAug = [xUpd;u];

Xc = repmat(xAug,1,nPts) + PAug_sqrt*CPtArray;

Xc_pred = zeros(3,nPts);

for pt = 1:nPts
    
    Xc(3,pt) = angleWrapping(Xc(3,pt));
    
    Xc_pred(:,pt) = processModel(Xc(1:nx,pt),Xc(nx+1:end,pt));
    
    Xc_pred(3,pt) = angleWrapping(Xc_pred(3,pt));

end;



xPred = sum(Xc_pred,2)/nPts; 
xPred(3) = angleWrapping(xPred(3));

X = (Xc_pred-repmat(xPred,1,nPts));
X(3) = angleWrapping(X(3));

PPred = X*X'/nPts;




