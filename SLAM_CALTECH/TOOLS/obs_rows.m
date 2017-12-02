function [ix, iy, ind] = obs_rows (observations)
%-------------------------------------------------------
%-------------------------------------------------------

ix = 2 * observations - 1;
iy = 2 * observations;
    
ind = reshape([ix ; iy], [], 1);
    
