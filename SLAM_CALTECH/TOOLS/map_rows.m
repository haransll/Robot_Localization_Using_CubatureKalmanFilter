function [ix, iy, ind = map_rows (features)
%-------------------------------------------------------
%-------------------------------------------------------

ix = 3 + 2 * features - 1;
iy = 3 + 2 * features;
    
ind = reshape([ix ; iy], [], 1);
    
