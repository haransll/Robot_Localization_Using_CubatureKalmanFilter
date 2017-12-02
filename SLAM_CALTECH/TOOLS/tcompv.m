%-------------------------------------------------------
% composes two vectors of transformations in
% vector form
%-------------------------------------------------------
function tac=tcompv(tab,tbc),

tac = [];
%-------------------------------------------------------
% tab is a simple transformation
%-------------------------------------------------------
if size(tab,1)==3,
   tac=zeros(size(tbc));
   for t=1:3:size(tbc,1),
      tac(t:t+2) = tcomp(tab,tbc(t:t+2));
   end
%-------------------------------------------------------
% tbc is a simple transformation
%-------------------------------------------------------
elseif size(tbc,1)==3,
   tac=zeros(size(tab));
   for t=1:3:size(tab,1),
      tac(t:t+2) = tcomp(tab(t:t+2),tbc);
   end
%-------------------------------------------------------
% they have the same dimensions
%-------------------------------------------------------
elseif size(tab,1)==size(tbc,1),
   tac=zeros(size(tab));
   for t=1:3:size(tab,1),
      tac(t:t+2) = tcomp(tab(t:t+2),tbc(t:t+2));
   end
else
   error('TCOMPV: tab and tab do not have the same dimensions!!!');
end;

