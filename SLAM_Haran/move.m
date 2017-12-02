function x_now = move(x_prev)

u = getRobotControl();

x_now = processModel(x_prev,u);


