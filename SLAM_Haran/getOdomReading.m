function xOdom_now = getOdomReading(xOdom_prev)

global sigma_u;

u = getRobotControl();

xOdom_now = processModel(xOdom_prev,u); % lets compose our state now
uNoise = sigma_u*randn(3,1); % Addin noise to odometry
xOdom_now = processModel(xOdom_now,uNoise); % add noise to the vector
