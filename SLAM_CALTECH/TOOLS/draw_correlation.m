function draw_correlation(P)
%-------------------------------------------------------
% University of Zaragoza
% Centro Politecnico Superior
% Robotics and Real Time Group
% Authors:  J. Neira, J. Tardos
% Date   :  7-2004
%-------------------------------------------------------
%-------------------------------------------------------

Corr = correlation(P);
figure;
%ncol=1000;
ncol=256 ;
mapa=hsv2rgb([linspace(2/3, 0, ncol)' 0.9*ones(ncol,1) ones(ncol,1)]) ;
mapa(1,:)=[0 0 0] ;
colormap(mapa) ;
%Esto dibuja en B/N
%imshow(abs(Corr)) ;
%Esto dibuja en colores frio--caliente
%s = sqrt(diag(Corr))' ;
%corr=diag(1./s)*Corr*diag(1./s) ;
imagesc(abs(Corr), [0 1]) ;  % asi no se ve el signo
%imagesc(Corr, [-1 1]) ; % Asi se ve e signo 
colorbar ;
