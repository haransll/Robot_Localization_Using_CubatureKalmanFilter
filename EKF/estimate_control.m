function u = estimate_control(xOdom_prev,xOdom_now)

theta = xOdom_prev(3);

c = cos(theta);
s = sin(theta);
Rinv = [c s 0;
        -s c 0;
        0 0 1];
    
delta_odom = xOdom_now - xOdom_prev;

delta_odom(3) = angleWrapping(delta_odom(3));
    
u = Rinv*delta_odom;

