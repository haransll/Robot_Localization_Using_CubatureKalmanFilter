
%function main_h()


close all;
clc;
clear all;


flg = 0;
nLmks = 10;



global sigma_mst;
global sigma_u;
global sigma_u_est;
global sigma_mst_est;
global lidar_bearing;
global lidar_range; %[m]
global map_lmk;

sigma_u = diag([1e-2,1e-2,1.5*pi/180]);
sigma_u_est = 2*sigma_u;

sigma_mst = diag([1.1, 5*pi/180]); %range and bearing
% sigma_mst = diag([1.1, 5*pi/180]); %range and bearing

sigma_mst_est = 2*sigma_mst;

lidar_bearing = 30*pi/180;
lidar_range = 50; %[m]


map_lmk = [0,10,1;
    0,50,2;
    50,50,3;
    50,10,4];
nLmks = size(map_lmk,1);

% map_lmk = [];
% for i=1:nLmks
%     
%     map_lmk = [map_lmk; 50*rand(1,2) i];
%     
% end;

WorldSize = 100;

nSteps = 240;

xRob_init = [ 40 25 0]'; % set init position (x, y, theta)

gt = [];
est = [];

xRob_gt_prev = xRob_init;

xUpd = xRob_init;
PUpd = diag([1,1,1e-2]);

xOdom_prev = xRob_init;

cnt_sense = 0;
lmks = [];
cnt_lmk = 0;
cnt_lmk_true = [];
cnt_lmk_est = [];

llmk_true = [];
llmk_est = [];
pos_err = [];
theta_err = [];

LMK_ID = [];
TT_LMK_ID = [];

for k=2:nSteps
    
    k
    
    xRob_gt_now = move(xRob_gt_prev); %Move
    
    [z_list,lmk_ids] = sense(xRob_gt_now);  %get range and bearing from landmarks
    
    
    %Estimation
    
    xOdom_now = getOdomReading(xOdom_prev);
    
    u_est = estimate_control(xOdom_prev,xOdom_now);
    
    %         u_est = getRobotControl();
    
    
    [xPred,PPred] = predict(xUpd,PUpd,u_est);
    
    
    
    if ~isempty(z_list)
        
        cnt_sense = cnt_sense+1;
        
        
        cnt_lmk_true = [cnt_lmk_true length(lmk_ids)];
        
        %predict landmarks
        
        tt_xRob = xRob_gt_now;
        
        tt_xLmk_obs = map_lmk(lmk_ids,1:2)';
        
        if flg == 0
            
            if k==61
                xx = 0;
                
            end;
            
            
%             est_lmk_ids = associate_msts_to_lmkID(xPred,z_list,tt_xRob,tt_xLmk_obs);
  
            [est_lmk_ids,obs_hyp] = probabilistic_data_association(xPred,PPred,z_list,tt_xRob,tt_xLmk_obs);

% est_lmk_ids = 1;
            
        else
            
            
%             est_lmk_ids = associate_msts_to_lmkID(xRob_gt_now,z_list,tt_xRob,tt_xLmk_obs);
           
            [est_lmk_ids,obs_hyp] = probabilistic_data_association(xRob_gt_now,z_list,tt_xRob,tt_xLmk_obs);
        end
        
        
        
        
        x = zeros(nLmks,1);
       if ~isempty(est_lmk_ids), x(est_lmk_ids) = 1; end;
       LMK_ID = [LMK_ID x];

        x = zeros(nLmks,1);
        x(lmk_ids) = 1;
        TT_LMK_ID = [TT_LMK_ID x];

        
        
        
        nObs = size(z_list,2);
        
        est_xLmk_list = [];
        
        xLmk_list = [];
        
        for m =1:nObs
            
            z_m = z_list(:,m);
            
            alpha = xRob_gt_now(3)+ z_m(2);
            
            est_xLmk_list = [est_xLmk_list, xRob_gt_now(1:2)+z_m(1)*[cos(alpha); sin(alpha)] ];  %transformed (to cartesian cordinate) in the gloabl FW
            
            xLmk_list = [xLmk_list, map_lmk(map_lmk(:,3)==lmk_ids(m),1:2)'];
       end
        
        
       
        est_xLmk_list
        xLmk_list
        
        
        
        cnt_lmk_est = [cnt_lmk_est length(est_lmk_ids)];
        
%         for m=1:nObs
%             
%                 [xUpd,PUpd] = EKF_update_v2(z_list(:,m),lmk_ids(m),xPred,PPred);
% 
%                 xPred = xUpd;
%                 PPred  = PUpd;
%             
%         end

        
        for m=1:nObs
            
            if obs_hyp(m)
                
                cnt_lmk = cnt_lmk+1;
                
                
                
                [xUpd,PUpd] = EKF_update_v2(z_list(:,m),est_lmk_ids(m),xPred,PPred);

                llmk_est = [llmk_est est_lmk_ids(1)];
                llmk_true = [llmk_true lmk_ids(1)];

                
                xPred = xUpd;
                PPred  = PUpd;
            else
                
                llmk_est = [llmk_est -1];
                llmk_true = [llmk_true lmk_ids(1)];
            
                xUpd = xPred;
                PPpd = PPred;
                
                
            end
            
        end

% % %         if ~isempty(est_lmk_ids)
% % %             
% % %             cnt_lmk = cnt_lmk+1;
% % %             
% % %             
% % % %             [xUpd,PUpd] = EKF_update(z_list,lmk_ids,xPred,PPred);
% % % 
% % %             [xUpd,PUpd] = EKF_update(z_list,est_lmk_ids,xPred,PPred);
% % % 
% % %             
% % %             
% % %             %             if flg == 0
% % %             %
% % %             %                 [xUpd,PUpd] = update_ckf(z_list,est_lmk_ids,xPred,PPred);
% % %             %             else
% % %             %                 [xUpd,PUpd] = update_ckf(z_list,lmk_ids,xPred,PPred);
% % %             %
% % %             %             end
% % %             
% % %             
% % %             
% % %             llmk_est = [llmk_est est_lmk_ids(1)];
% % %         else
% % %             
% % %             llmk_est = [llmk_est -1];
% % %         end
        
        
    else
        xUpd = xPred;
        PUpd = PPred;
        
        cnt_lmk_true = [cnt_lmk_true 0];
        cnt_lmk_est = [cnt_lmk_est 0];
        
        %         llmk_true = [llmk_true -1];
        
    end;
    pos_err =  [pos_err norm(xRob_gt_now(1:2) -xUpd(1:2))];
    theta_err = [theta_err (xRob_gt_now(3)-xUpd(3))];
    
    
    xOdom_prev = xOdom_now;
    xRob_gt_prev = xRob_gt_now;
    
    
    gt = [gt xRob_gt_now];
    est = [est xUpd];
    
end;

figure;
subplot(2,1,1);
plot(gt(1,:),gt(2,:),'r')
hold on;
plot(est(1,:),est(2,:),'b:')
plot(map_lmk(:,1),map_lmk(:,2),'b+');
set(gcf,'doublebuffer','on');
hLine = line([0,0],[0,0]);
% axis([ -10 60 -10 60]);
xlabel(' Initial Conditions and beacons at blue +');

%pause;


subplot(2,1,2);
plot(llmk_true,'r');
hold on;
plot(llmk_est,'b:');


figure;
plot(cnt_lmk_true,'r');
hold on;
plot(cnt_lmk_est,'b:');




figure;
subplot(2,1,1)
plot(pos_err);
subplot(2,1,2);
plot(180/pi*theta_err);







