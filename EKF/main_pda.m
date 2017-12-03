
%%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
%%% Author: Haran Arasaratnam
%%% Date 11/26/2017
%%% Revision: 
%%% added PDA for data association
%%% created a seperate m file for mahalanobis dist
%%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

close all;
clc;
clear all;

global sigma_mst;
global sigma_u;
global sigma_u_est;
global sigma_mst_est;
global lidar_bearing;
global lidar_range; %[m]
global map_lmk;

sigma_u = diag([1e-2,1e-2,1.5*pi/180]);
sigma_u_est = 1.2*sigma_u;

sigma_mst = diag([1.1, 5*pi/180]); %range and bearing

sigma_mst_est = 1.2*sigma_mst;

lidar_bearing = 30*pi/180;
lidar_range = 50; %[m]


map_lmk = [0,10,1;
    0,50,2;
    50,50,3;
    50,10,4];

% map_lmk = [];
% for i=1:nLmks
%
%     map_lmk = [map_lmk; 50*rand(1,2) i];
%
% end;


nLmks = size(map_lmk,1);

WorldSize = 100;

nSteps = 240;

xRob_init = [40 25 0]'; % set init position (x, y, theta)

xList_gt = [];  %ground truth
xList_est = []; %filter estimated state

xRob_gt_prev = xRob_init;

xUpd = xRob_init;
PUpd =diag([1,1,1e-2]);

xOdom_prev = xRob_init;

pos_err = [];
theta_err = [];

for k=2:nSteps
    
    xRob_gt_now = move(xRob_gt_prev);
    
    [z_list,lmk_ids] = sense(xRob_gt_now);  %get range and bearing from landmarks
    
    
    %Estimate
    
    xOdom_now = getOdomReading(xOdom_prev);
    
    u_est = estimate_control(xOdom_prev,xOdom_now);    
    
    [xPred,PPred] = predict_ekf(xUpd,PUpd,u_est);    
    
    if ~isempty(z_list)        
        
        [est_lmk_ids,obs_hyp] = probabilistic_data_association(xPred,PPred,z_list);        
        
        nObs = size(z_list,2);
        
        est_xLmk_list = [];
        
        xLmk_list = [];
        
        for m =1:nObs
            
            z_m = z_list(:,m);
            
            if obs_hyp(m)                               
                
                [xUpd,PUpd] = update_ekf(z_list(:,m),est_lmk_ids(m),xPred,PPred);
                
                xPred = xUpd;
                PPred  = PUpd;
            else
                
                xUpd = xPred;
                PUpd = PPred;
                
                
            end
            
            
            alpha = xRob_gt_now(3)+ z_m(2);
            
            est_xLmk_list = [est_xLmk_list, xRob_gt_now(1:2)+z_m(1)*[cos(alpha); sin(alpha)] ];  %transformed (to cartesian cordinate) in the gloabl FW
            
            xLmk_list = [xLmk_list, map_lmk(map_lmk(:,3)==lmk_ids(m),1:2)'];            
            
        end
        
        
    else
        
        xUpd = xPred;
        PUpd = PPred;        
        
    end;
    
    xOdom_prev = xOdom_now;
    xRob_gt_prev = xRob_gt_now;    
    
    
    xList_gt = [xList_gt xRob_gt_now];
    xList_est = [xList_est xUpd];
    
    pos_err =  [pos_err norm(xRob_gt_now(1:2) -xUpd(1:2))];
    theta_err = [theta_err (xRob_gt_now(3)-xUpd(3))];    
    

    
end;

figure;
subplot(3,1,1);
plot(xList_gt(1,:),xList_gt(2,:),'r','linewidth',1)
hold on;
plot(xList_est(1,:),xList_est(2,:),'b:','linewidth',1)
plot(map_lmk(:,1),map_lmk(:,2),'b+','linewidth',2);
set(gcf,'doublebuffer','on');
hLine = line([0,0],[0,0]);
axis([ -10 60 -10 60]);
xlabel('Landmarks at blue +');
legend('True','estimated');

subplot(3,1,2)
plot(pos_err, 'linewidth',1);
ylabel('error in pos [m]');
grid on;

subplot(3,1,3);
plot(180/pi*theta_err,'linewidth',1);
ylim([-10 10]);
ylabel('error in heading angle [deg]');
grid on;

% figure
% subplot(2,1,2);
% plot(llmk_true,'r');
% hold on;
% plot(llmk_est,'b:');
%
%
% figure;
% plot(cnt_lmk_true,'r');
% hold on;
% plot(cnt_lmk_est,'b:');







