function [xUpd,PUpd] = update_ekf(z,est_lmk_id,xPred,PPred)

global map_lmk;
global sigma_mst_est;



xLmk = map_lmk(map_lmk(:,3)==est_lmk_id,1:2)';

zPred = measModel(xPred,xLmk);

jH = zeros(2,3);
Delta = (xLmk-xPred(1:2));
r = norm(Delta);
jH(1,1) = -Delta(1) / r;
jH(1,2) = -Delta(2) / r;
jH(2,1) = Delta(2) / (r^2);
jH(2,2) = -Delta(1) / (r^2);
jH(2,3) = -1;

S = jH*PPred*jH'+sigma_mst_est.^2;
W = PPred*jH'*pinv(S);

innov = z - zPred;
innov(2) = angleWrapping(innov(2));

xUpd = xPred+ W*innov;

PUpd = PPred - W*S*W';
PUpd = 0.5*(PUpd+PUpd');