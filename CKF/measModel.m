function z = measModel(X,xLmk)


dx = xLmk(1) - X(1,:);
dy = xLmk(2) - X(2,:);
d2= dx.^2 + dy.^2;
d= sqrt(d2);

alpha =  atan2(dy,dx);

z = [d;
    (alpha-X(3,:))];

z(2,:) = angleWrapping(z(2,:));


