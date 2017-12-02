function Corr=correlation(Cov)
% Obtiene la matriz de correlacion a partir de la Covarianza

sigmas = sqrt(diag(Cov))' ;
Corr=diag(1./sigmas)*Cov*diag(1./sigmas) ;

