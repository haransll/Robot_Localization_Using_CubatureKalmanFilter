%% Function to handle the transformation of angle
function angle = angleWrapping(angle)

while (angle>pi), angle=angle-2*pi; end;
    
while (angle<-pi), angle = angle+2*pi; end;
    
