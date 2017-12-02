function compatibility = compute_compatibility (prediction, observations)
%-------------------------------------------------------
%-------------------------------------------------------
global chi2;

% Compute individual distances
compatibility.d2 = zeros(observations.m, prediction.n);

for i = 1:observations.m
     [ix, iy, indi] = obs_rows(i);
     z = observations.z(indi);
     R = observations.R(indi,indi);
     for j = 1:prediction.n
         [jx, jy, indj] = obs_rows(j);
         e = z - prediction.h(indj);
         C = prediction.HPH(indj,indj) + R;
         compatibility.d2(i,j) = mahalanobis(e, C);
     end
end

compatibility.ic = compatibility.d2 < chi2(2);
compatibility.candidates.features = find(sum(compatibility.ic, 1));
compatibility.candidates.observations = find(sum(compatibility.ic, 2))';

compatibility.AL = (sum (compatibility.ic, 2))';
compatibility.HS = prod(compatibility.AL + 1);

