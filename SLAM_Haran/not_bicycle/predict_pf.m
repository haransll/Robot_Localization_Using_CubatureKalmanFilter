function [xP] = predict_pf(xP,u)

global sigma_u_est;
global nP;

for i= 1:nP
    
    u_noisy = u+sigma_u_est*randn(3,1);
    
    
    xP(:,i) = processModel(xP(:,i),u_noisy); 


end


