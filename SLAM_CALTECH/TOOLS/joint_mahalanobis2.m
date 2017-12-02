function [d2k, Hk, Ck, hk, zk, Rk] = joint_mahalanobis2 (prediction, observations, H)
%-------------------------------------------------------
%-------------------------------------------------------

% Compute joint distance for a hypothesis
[kk,i, j] = find(H);

[ix, iy, indi] = obs_rows(i);
[jx, jy, indj] = obs_rows(j);

zk = observations.z(indi);
hk = prediction.h(indj);
Rk = observations.R(indi,indi);
Ck = prediction.HPH(indj,indj) + Rk;
Hk = prediction.H(indj,:);
d2k = mahalanobis (zk - hk, Ck);
