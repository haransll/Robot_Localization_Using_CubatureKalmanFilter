function d2 = mahalanobis (z, P)
global calculations;
%-------------------------------------------------------
% 
%-------------------------------------------------------

Cp = chol(P);
y = Cp'\z;
d2 = full(y'*y);
