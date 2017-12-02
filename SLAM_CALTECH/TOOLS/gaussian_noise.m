function x = gaussian_noise(P)
%-------------------------------------------------------
% University of Zaragoza
% Centro Politecnico Superior
% Robotics and Real Time Group
% Authors:  J. Neira, J. Tardos
% Date   :  7-2002
%-------------------------------------------------------
% function x = gaussian_noise(P)
%
% Generate a Gaussian noise x following N(0,P).
%-------------------------------------------------------%
[V,D] = eig(P);
x = V*(randn(size(D,1),1).*sqrt(diag(D)));