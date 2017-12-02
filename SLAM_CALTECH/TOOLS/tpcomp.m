%-------------------------------------------------------
% Composition of a transformation and a set of points
%-------------------------------------------------------
function Pa = tpcomp(Tab, Pb),
s = sin(Tab(3));
c = cos(Tab(3));
Pa = [Tab(1) + [c -s]*Pb
      Tab(2) + [s  c]*Pb];
