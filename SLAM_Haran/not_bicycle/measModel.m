function z = measModel(xRob,xLmk)


delta = xLmk - xRob(1:2);

range = norm(delta);

alpha =  atan2(delta(2),delta(1));

bearing = angleWrapping(alpha-xRob(3));

z = [range;bearing];


