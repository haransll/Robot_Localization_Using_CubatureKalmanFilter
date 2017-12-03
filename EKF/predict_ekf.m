function [xPred,PPred] = predict_ekf(xUpd,PUpd,u)

global sigma_u_est;

xPred = processModel(xUpd,u); 

theta = xUpd(3);
s = sin(theta);
c = cos(theta);

J2 = [c -s 0;
     s c 0;
     0 0 1];  % rotation matrix
 

%J1 is jacobian  
 J1 = [1 0 -u(1)*s-u(2)*c;
      0 1 u(1)*c-u(2)*s;
      0 0 1];


PPred = J1*PUpd*J1' + J2*(sigma_u_est.*2)*J2';



