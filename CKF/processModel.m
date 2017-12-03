function xv = processModel(x,u)

theta = x(3,:);
s = sin(theta);
c = cos(theta);


xv = [x(1,:)+c.*u(1,:)-s.*u(2,:);
    x(2,:)+s.*u(1,:)+c.*u(2,:);
    x(3,:)+u(3,:)];

xv(3,:) = angleWrapping(xv(3,:));

