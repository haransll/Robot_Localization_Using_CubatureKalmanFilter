function DoVehicleGraphics(x,P,nSigma,Forwards)
ShiftTheta = atan2(Forwards(2),Forwards(1));
h = PlotEllipse(x,P,nSigma);
if(~isempty(h))
    set(h,'color','r');
end;
DrawRobot(x,'b',ShiftTheta);

