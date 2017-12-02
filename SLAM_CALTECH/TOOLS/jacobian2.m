%-------------------------------------------------------
% calculates the J2 jacobian of a pair of transformation
%-------------------------------------------------------
function J = jacobian2(tab, tbc)

if length(tab) ~= 3,
   error('Jacobian1: tab is not a transformation!!!');
end

if length(tab) ~= 3,
   error('Jacobian1: tbc is not a transformation!!!');
end

p1 = tab(3);

J = [cos(p1) -sin(p1)  0
     sin(p1)  cos(p1)  0
     0          0      1];
