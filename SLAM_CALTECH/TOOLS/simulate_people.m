%-----------------------------------------
function [xnew, ynew] = simulate_people (n, xmin, xmax, ymin, ymax),

persistent x  y  vx  vy;

speed = 0.05;
accel = 0.01;

if nargin == 5
     x = xmin + (xmax-xmin)* rand(n,1);
     y = ymin + (ymax-ymin)* rand(n,1);
     vx = speed * randn(n,1);
     vy = speed * randn(n,1);
else
     x = x + vx;
     y = y + vy;
     vx = vx + accel * randn(size(vx));
     vy = vy + accel * randn(size(vy));
end

xnew = x;
ynew = y;
