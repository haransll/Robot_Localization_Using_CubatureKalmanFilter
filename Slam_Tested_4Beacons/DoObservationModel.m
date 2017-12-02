
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [z] = DoObservationModel(xVehicle, xFeature)
% get the distance to the landmarks
DeltaObs = xFeature-xVehicle(1:2);
z = [norm(DeltaObs); % normalize (x,y)
    atan2(DeltaObs(2),DeltaObs(1))-xVehicle(3)]; % get the angle 
z(2) = AngleWrapping(z(2)); % handle with change of quadrant
end