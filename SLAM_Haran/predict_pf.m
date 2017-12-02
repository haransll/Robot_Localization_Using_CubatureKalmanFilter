function [xP, xPred,PPred] = predict_pf(xP,u,weights)

global sigma_u_est;
global num_particles;

for iP= 1:num_particles
    
    u_noisy = u+sigma_u_est*randn(3,1);
    
    
    xP(:,iP) = processModel(xP(:,iP),u_noisy); 



end

% xPred = weights.*
% PPred = J1*PUpd*J1' + J2*(sigma_u_est.*2)*J2';
