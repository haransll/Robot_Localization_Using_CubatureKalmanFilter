function xv = processModel(x,u)

% theta = xRob_prev(3);
% s = sin(theta);
% c = cos(theta);
% R = [c -s;
%     s c];  % rotation matrix
% 
% headAngle = xRob_prev(3)+u(3);
% 
% headAngle = angleWrapping(headAngle);
% 
% xRob_now = [xRob_prev(1:2)+R*u(1:2);headAngle];

dt = 1;
V = u(1); %wheel vel
G = u(2); % steering angle
WB = 4;
xv= [x(1,:) + V.*dt.*cos(G+x(3,:)); 
     x(2,:) + V.*dt.*sin(G+x(3,:));
     x(3,:) + V.*dt.*sin(G)/WB];
 
xv(3,:) = angleWrapping(xv(3,:));
