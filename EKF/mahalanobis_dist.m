function d2 = mahalanobis_dist(z,xRob,xLmk,PPred)

global sigma_mst_est;

zPred = measModel(xRob,xLmk);

jH = zeros(2,3);
Delta = (xLmk-xRob(1:2));
r = norm(Delta);
jH(1,1) = -Delta(1) / r;
jH(1,2) = -Delta(2) / r;
jH(2,1) = Delta(2) / (r^2);
jH(2,2) = -Delta(1) / (r^2);
jH(2,3) = -1;


innov = (z - zPred);
innov(2) = angleWrapping(innov(2));

S = jH*PPred*jH'+sigma_mst_est.^2;
Sc = chol(S);
y = Sc'\innov;
d2 = (y'*y);
