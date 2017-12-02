%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [xnow] = GetOdometry(CtrlNoise)
% This can save internal, similar to static, avoids store the value and
% making blobal
persistent LastOdom; 
global UTrue;

% Firt run? inicialize with xVehicleTrue
if(isempty(LastOdom))
    global xVehicleTrue;
    LastOdom = xVehicleTrue;
end;
u = GetRobotControl();
xnow =tcomp(LastOdom,u); % lets compose our state now
uNoise = sqrt(UTrue)*CtrlNoise; % Addin noise to odometry
xnow = tcomp(xnow,uNoise); % add noise to the vector
LastOdom = xnow;  