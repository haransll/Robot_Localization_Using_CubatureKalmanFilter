%% Extended kalman filter
function EKFSLAM
% Clear all
close all; clear all;
% Global variables (avoid extra copies)
global xVehicleTrue;
global LandFeatures;
global LaserSensorSettings;
global VisionSensorSettings; % for latter incorporation id possible
global xOdomLast;
global nSteps;
global UTrue;

% Setting of the sensors specs
LaserSensorSettings.Bearing =30; % Degrees
LaserSensorSettings.Range =50; % Meters
display(sprintf('=== Sensor Definitions ===  \n Max Range:  %d Meters\n Max Bearing: %d Degrees \n========================== \n',LaserSensorSettings.Range,LaserSensorSettings.Bearing));
display(' Paused!!! Press any Key!! ');
pause;


% Length of the experiment
nSteps =10000; % change to extend the lentgh of the simulation

% Size Map
WorldSize =200;

% Number of landfeatures to add to map
nLandFeatures =4;

% Make some spare for then
LandFeatures = zeros(2,1,nLandFeatures);

% Lets put the beacons at desired positions ( need to match the number
% defined)!!!! if nor some of then cannot be scanned!!
LandFeatures(:,:,1)=[0 10]';
LandFeatures(:,:,2)=[0 50]';
LandFeatures(:,:,3)=[50 50]';
LandFeatures(:,:,4)=[50 10]';

% Initial position of the vehicle
xVehicleTrue = [ 40 25 0]'; % start position (x, y, theta)

% Set the initial conditions of the filter
xEst =[xVehicleTrue]; % already an columm vector
PEst = diag([1 1 0.01]); % start with an medium covar ( Atention EKF need an good initialization)

% Make some space to save the Landmarks detected (arraylist (x,y)
MappedLandFeatures = NaN*zeros(nLandFeatures,2);


% Additive noise to be added at the observation
obsNoise = randn(2,1)

% Ok lets show where are our Landmarks located in the world
figure(1); hold on; grid off; axis equal;
plot(LandFeatures(1,:),LandFeatures(2,:),'b+');hold on;
set(gcf,'doublebuffer','on');
hLine = line([0,0],[0,0]);
set(hLine,'linestyle',':');
axis([ -WorldSize/2 WorldSize/2 -WorldSize/2 WorldSize/2]);
xlabel(' Initial Conditions and beacons at blue +');
display(sprintf('\n\n\n Showing initial Location \n Press any key to procced\n'));
pause;

% Standart deviation errors added to control
UTrue = diag([0.01,0.01,1.5*pi/180]).^2;

% Standard deviation errors regarding observation
RTrue = diag([1.1,5*pi/180]).^2;


% Aditive factor for control estimation and noise
UEst = 2.0*UTrue;
REst = 2.0*RTrue;



% All Setup and ready to go
% Extract first odometry for predict
CtrlNoise = randn(3,1); % some random control noise to be added ( note change to gaussian)
xOdomLast = GetOdometry(CtrlNoise);

for k = 2:nSteps
    
    % get robot Control and simulate the movement
    CtrlNoise = randn(3,1); % some random control noise to be added
    SimulateMovement(CtrlNoise);
    
    xOdomNow = GetOdometry(CtrlNoise); % Get the odometry values c
    u = tcomp(tinv(xOdomLast),xOdomNow); % figure out control 
    xOdomLast = xOdomNow; % refhesh odometry vector
    
    % extract the two compoments of the estimation ( Vehicle and
    % landfeatures
    % get vehicle estimation
    xVehicle = xEst(1:3); % estimated only the part for the vehicle
    % get landferatures estimation
    xMap = xEst(4:end); % Landfeatures estimation are the rest ones
 
    
    %do prediction (the following is simply the result of multiplying 
    %out block form of jacobians)     
    xVehiclePred = tcomp(xVehicle,u); % estimation is regarding current and control vector
    % covariance for vehicle
    %J2 is just an simple rotation matrix over x
    % just need to change to the cnematic model and speeds
    
    % Change this!!!
    PPredvv = J1(xVehicle,u)* PEst(1:3,1:3) *J1(xVehicle,u)' + J2(xVehicle,u)* UEst * J2(xVehicle,u)';
    % vehicle/ landmarks,
    PPredvm = J1(xVehicle,u)*PEst(1:3,4:end);
    % For the rest just extract (landfeatures
    PPredmm = PEst(4:end,4:end);
    
    % agregates robot prediction and beacons
    xPred = [xVehiclePred;xMap];
%     % Compose the covariance matrix
%          Ppred = |Pvv Pvm
%                  |Pvm' Pmm
    PPred = [PPredvv PPredvm;
        PPredvm' PPredmm];
       

    % Get the observation (passing noise to be added and Noise true
    ObsNoise =randn(2,1);
    [z,iFeature] = GetObservation(ObsNoise,RTrue);
    
   
     if(~isempty(z)) % if i get an valid observation
        %have we seen this feature before?
        % need to change here ( see Validation gate White)
        if( ~isnan(MappedLandFeatures(iFeature,1)))
            
            %predict observation: find out where it is in state vector
            FeatureIndex = MappedLandFeatures(iFeature,1);
            % Extract feature values in state vector
            xFeature = xPred(FeatureIndex:FeatureIndex+1);
            
            % predict the observation
            zPred = DoObservationModel(xVehicle,xFeature);
            
            % get observation Jacobians
            [jHxv,jHxf] = GetObsJacs(xVehicle,xFeature);
            
            % fill in state jacobian
            jH = zeros(2,length(xEst));
            jH(:,FeatureIndex:FeatureIndex+1) = jHxf;
            jH(:,1:3) = jHxv;
            
            %do Kalman update:
            % check for inovation (here we can use the validation gate
            Innov = z-zPred;
            % angles have always to be checked only betwewn 0 an 2Pi
            Innov(2) = AngleWrapping(Innov(2));
            
            S = jH*PPred*jH'+REst;
            W = PPred*jH'*inv(S); 
            xEst = xPred+ W*Innov;
            
            PEst = PPred-W*S*W';
            
            %ensure P remains symmetric
            PEst = 0.5*(PEst+PEst');
        else
            % this is a new feature
            
            % extract length of the current tracking landmarks
            nStates = length(xEst); 
            
            % Compute the x,y position of the landmark x,y robot plus
            % Xxvehicle +distance*cos(angle + Xthvehicle)
            % Xyvehicle + distance* sin(angle + Xthvehicle)
            xFeature = xVehicle(1:2)+ [z(1)*cos(z(2)+xVehicle(3));z(1)*sin(z(2)+xVehicle(3))];
            xEst = [xEst;xFeature]; %add to state vector estimated
            % compute jacobians regaing feature and observation 
            [jGxv, jGz] = GetNewFeatureJacs(xVehicle,z); 
            
            % M=| 1 0 0  0 0
            %   | 0 1 0  0 0
            %   | 0 0 1  0 0
            %   -------------
            %   |
            
            M = [eye(nStates), zeros(nStates,2);% no use jacobian w.r.t vehicle
                jGxv zeros(2,nStates-3)  , jGz];
            
            PEst = M*blkdiag(PEst,REst)*M';
            
            %remember this feature as being mapped we store its ID and position in the state vector
            MappedLandFeatures(iFeature,:) = [length(xEst)-1, length(xEst)];
            
        end;
     else
        % notinng new found lets procced refreshing the values for next run
        xEst = xPred;
        PESt = PPred;
    end;
    
    
    % at all interactions
    
    % Plot the veicule shape LOL
        a = axis;
        clf;
        axis(a);hold on;
        n  = length(xEst); % get the total state and vector also
        % only the landmarks
        nF = (n-3)/2;
        
        % Draw vehicle and its eliipse only the 3 first are the vehicle
%         % state   P = |v v v|-----
%                       |v v v|-----
%                       |v v v|-----
%                       ------------
        DoVehicleGraphics(xEst(1:3),PEst(1:3,1:3),3,[0 1]);
        
        % if we get an valid observation plot is values
        if(~isnan(z))
            % Plot the line to beacon
            h = line([xEst(1),xFeature(1)],[xEst(2),xFeature(2)]);
            set(h,'linestyle',':');
            %make some visible information
            legend( sprintf('Range: %3.2fm; Angle: %3.2f: °  ',z(1),(z(2)-pi/2)*180/pi));
            
        end;
        % For all plot the covariance elipse of each state fr the landmark
        for(i = 1:nF)
            iF = 3+2*i-1; 
            plot(xEst(iF),xEst(iF+1),'b*');
            PlotEllipse(xEst(iF:iF+1),PEst(iF:iF+1,iF:iF+1),3);
        end;        
        k      
        drawnow;  
end
end












