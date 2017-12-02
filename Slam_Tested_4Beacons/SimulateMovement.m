%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function SimulateMovement(CtrlNoise)
global xVehicleTrue;
u = GetRobotControl();
xVehicleTrue = tcomp(xVehicleTrue,u);