function z = measModel(xRob,xLmk)


delta = xLmk - xRob(1:2);

range = norm(delta);

bearing = angleWrapping( atan2(delta(2),delta(1))-xRob(3) );

z = [range;bearing];
