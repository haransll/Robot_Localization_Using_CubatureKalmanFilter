function h = draw_ellipse(pos, cov, color)
%-------------------------------------------------------
% University of Zaragoza
% Centro Politecnico Superior
% Robotics and Real Time Group
% Authors:  J. Neira, J. Tardos
% Date   :  7-2004
%-------------------------------------------------------
%-------------------------------------------------------
global chi2;

persistent CIRCLE

if isempty(CIRCLE) 
    tita = linspace(0, 2*pi,40);
    CIRCLE = [cos(tita); sin(tita)];
end

[V,D]=eig(full(cov(1:2,1:2)));
ejes=sqrt(chi2(2)*diag(D));
P = (V*diag(ejes))*CIRCLE;
hp = line(P(1,:)+pos(1), P(2,:)+pos(2));
set(hp,'Color', color);
%set(hp, 'LineWidth', 1.5);
