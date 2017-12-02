function xRob_now = processModel(xRob_prev,u)

theta = xRob_prev(3);
s = sin(theta);
c = cos(theta);
R = [c -s;
    s c];  % rotation matrix

headAngle = xRob_prev(3)+u(3);

headAngle = angleWrapping(headAngle);

xRob_now = [xRob_prev(1:2)+R*u(1:2);headAngle];
