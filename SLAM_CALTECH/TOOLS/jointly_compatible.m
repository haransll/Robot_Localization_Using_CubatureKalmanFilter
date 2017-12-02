function answer = jointly_compatible (prediction, observations, H)
%-------------------------------------------------------
% University of Zaragoza
% Centro Politecnico Superior
% Robotics and Real Time Group
% Authors:  J. Neira, J. Tardos
% Date   :  7-2004
%-------------------------------------------------------
%-------------------------------------------------------
global chi2;

d2 = joint_mahalanobis2 (prediction, observations, H);
dof = 2*length(find(H));
answer = d2 < chi2(dof);


