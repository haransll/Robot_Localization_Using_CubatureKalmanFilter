function [tac, Pac] = ucomp(tab, Pab, tbc, Pbc);
%-------------------------------------------------------
% University of Zaragoza
% Centro Politecnico Superior
% Robotics and Real Time Group
% Authors:  J. Neira, J. Tardos
% Date   :  7-2004
%-------------------------------------------------------
%-------------------------------------------------------

tac = tcomp(tab, tbc);
J1 = jacobian1(tab, tbc);
J2 = jacobian2(tab, tbc);
Pac = J1 * Pab * J1' + J2 * Pbc * J2';
