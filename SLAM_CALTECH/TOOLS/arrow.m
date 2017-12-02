function arrow(initial_point, end_point, c)
%-------------------------------------------------------
% University of Zaragoza
% Centro Politecnico Superior
% Robotics and Real Time Group
% Author :  P. Pinies
% Date   :  7-2004
%-------------------------------------------------------
%   arrow(initial_point, end_point, c) where
%   Input:
%   ------
%   -initial_point: 2D vector 
%   -end_point: 2D vector
%   -c: color
%-------------------------------------------------------

x_dif=end_point(1)-initial_point(1);
y_dif=end_point(2)-initial_point(2);
modulus=sqrt(x_dif^2 + y_dif^2);
phi=atan2(y_dif, x_dif);
x=initial_point(1);
y=initial_point(2);

arrow=[0 1 0.7  0.7   
       0 0 0.1 -0.1  
       1 1  1    1 ];

arrow(1:2,:)=arrow(1:2,:)*modulus;       

X_WP=[cos(phi) -sin(phi)  x
    sin(phi)  cos(phi)  y
    0         0      1];

arrow=X_WP*arrow;
arrow=arrow(1:2,:)';

h = plot(arrow([1,2],1),arrow([1,2],2),c);
set(h, 'LineWidth', 1.5);
h = plot(arrow([2,3],1),arrow([2,3],2),c);
set(h, 'LineWidth', 1.5);
h = plot(arrow([2,4],1),arrow([2,4],2),c);
set(h, 'LineWidth', 1.5);

return;