function tac=tcomp(tab,tbc)


% final angle
result = tab(3)+tbc(3); % add the angles

if result > pi || result <= -pi
   result = AngleWrapping(result) ;
end

s = sin(tab(3));
c = cos(tab(3));

% actual value added with the new control vector
tac = [tab(1:2)+ [c -s; s c]*tbc(1:2);
       result];