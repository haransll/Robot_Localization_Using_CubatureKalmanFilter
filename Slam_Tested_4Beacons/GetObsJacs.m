%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [jHxv,jHxf] = GetObsJacs(xPred, xFeature)
jHxv = zeros(2,3);

jHxf = zeros(2,2);
Delta = (xFeature-xPred(1:2));
r = norm(Delta);
jHxv(1,1) = -Delta(1) / r;
jHxv(1,2) = -Delta(2) / r;
jHxv(2,1) = Delta(2) / (r^2);
jHxv(2,2) = -Delta(1) / (r^2);
jHxv(2,3) = -1;
jHxf(1:2,1:2) = -jHxv(1:2,1:2);