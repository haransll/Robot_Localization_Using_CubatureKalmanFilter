close all;
clear all;
P = diag([1^2 2^2 (10*pi/180)^2])

figure
axis equal
hold on

for i = 1:1000
    [V,D] = eig(P);
    x = V*(randn(size(D,1),1).*sqrt(diag(D)));
    plot3(x(1),x(2),x(3),'b.');
end
chi = 7.82;
%[x,y,z] = ellipsoid(0,0,0,sqrt(chi*P(1,1)), sqrt(chi*P(2,2)), sqrt(chi*P(3,3)),10);
%mesh(x,y,z)
%hidden off