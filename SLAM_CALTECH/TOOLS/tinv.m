%-------------------------------------------------------
% University of Zaragoza
% Centro Politecnico Superior
% Robotics and Real Time Group
%-------------------------------------------------------
% Authors:  J. Neira, J. Tardos
% Date   :  7-2002
%
function tba = tinv (tab)
%
% calculates the inverse of one or more transformations
%-------------------------------------------------------

tba = zeros(size(tab));
for t=1:3:size(tab,1),
   tba(t:t+2) = tinv1(tab(t:t+2));
end

%-------------------------------------------------------
function tba = tinv1 (tab)
%
% calculates the inverse of one transformations
%-------------------------------------------------------
s = sin(tab(3));
c = cos(tab(3));
tba = [-tab(1)*c - tab(2)*s;
        tab(1)*s - tab(2)*c;
       -tab(3)];


