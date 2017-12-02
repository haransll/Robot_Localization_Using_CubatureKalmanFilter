function observations = get_observations (ground, sensor, step)
%-------------------------------------------------------
% University of Zaragoza
% Centro Politecnico Superior
% Robotics and Real Time Group
% Authors:  J. Neira, J. Tardos
% Date   :  7-2004
%-------------------------------------------------------
%-------------------------------------------------------
global configuration;
global people;

twr = ground.trajectory(end).x;
trw = tinv(twr);

if configuration.people
    xp = people.gx(step,:);
    yp = people.gy(step,:);
    people.x = [people.x; xp']; people.y = [people.y; yp'];
else
    xp = []; yp = [];
end

twp = [ground.points [xp; yp]];
trp = tpcomp(trw, twp);


x = trp(1:2:end);
y = trp(2:2:end);

[tita, rho] = cart2pol (x, y);

visible = find((rho < sensor.range) & (tita >= sensor.minangle) & (tita <= sensor.maxangle));

xv = x(visible);
yv = y(visible);
rhov = rho(visible);
titav = tita(visible);
observations.ground = [xv; yv];

srho = rhov * sensor.srho;
stita = ones(size(titav)) * sensor.stita;
xn = [];
yn = [];
Rn = [];

for i=1:length(srho),
    Ri = diag([srho(i)^2 stita(i)^2]);
    
    Ji = [cos(titav(i)) -rhov(i)*sin(titav(i))
        sin(titav(i))  rhov(i)*cos(titav(i))];
    
    if configuration.noise
        ni = gaussian_noise(Ri);
    else
        ni = [ 0; 0];
    end
    
    rhoi = rhov(i) + ni(1);
    titai = titav(i) + ni(2);
    [xi, yi] = pol2cart(titai,rhoi);
    xn = [xn; xi];
    yn = [yn; yi];
    Ri = Ji * Ri * Ji';  
    Rn = blkdiag(Rn, Ri);
end


trp = [xn'; yn'];
trp = reshape(trp, [], 1);

observations.m = length(srho);
observations.z = trp;
observations.R = Rn;
observations.ground_id = visible .* [visible <= ground.n];
