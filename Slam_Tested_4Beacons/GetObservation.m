%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [z,iFeature] = GetObservation(ObsNoise,RTrue)
global LandFeatures;global xVehicleTrue;global LaserSensorSettings

z =[];

iFeature = -1; % return negative, no detection at all by default

% get all landfeatures and tested each
l =length(LandFeatures); % get amount of landmarks
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% NOTE: Need to handle several beacons if they are close ( for example save
% the closest one or return an struct with all detected on range and handle
% all. in this case return the first one that sees LOL
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% For all landfeatures
for i=1 :l
     
    % Lets measure the LandFeatures and add some aditive noise
    zTemp = DoObservationModel(xVehicleTrue,LandFeatures(:,i))+sqrt(RTrue)*ObsNoise;
    
    % Need to see witch quadrant angle (passing angle
    zTemp(2) = AngleWrapping(zTemp(2));
    %Observation model of the sensor ( Bearing and range
    if(abs(pi/2-zTemp(2))<LaserSensorSettings.Bearing*pi/180 & zTemp(1) < LaserSensorSettings.Range)
        z=zTemp; % copy calculated observation
        iFeature =i; % Return lable of the feature
        display(sprintf(' LandMark Found !! ID:%d \n',i));
        break; % exit with some value, else empty
    end
end;