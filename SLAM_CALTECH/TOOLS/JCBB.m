%-------------------------------------------------------
function H = JCBB (prediction, observations, compatibility)
% 
%-------------------------------------------------------
global Best;
global configuration;

Best = zeros(1, observations.m);

JCBB_R (prediction, observations, compatibility, [], 1);

H = Best;
configuration.name = 'JOINT COMPATIBILITY B & B';

%-------------------------------------------------------
function JCBB_R (prediction, observations, compatibility, H, i)
% 
%-------------------------------------------------------
global Best;
global configuration;

if i > observations.m % leaf node?
    if pairings(H) > pairings(Best) % did better?
        Best = H;
    end
else
    individually_compatible = find(compatibility.ic(i,:));
    for j = individually_compatible
        if jointly_compatible(prediction, observations, [H j])
            JCBB_R(prediction, observations, compatibility, [H j], i + 1); %pairing (Ei, Fj) accepted 
        end
    end
%    if pairings(H) + length(compatibility.candidates.observations) - i >= pairings(Best) % can do better?
%    if pairings(H) + observations.m - i >= pairings(Best) % can do better?
    if pairings(H) + pairings(compatibility.AL(i+1:end)) >= pairings(Best) % can do better?
        JCBB_R(prediction, observations, compatibility, [H 0], i + 1); % star node: Ei not paired
    end
end

%-------------------------------------------------------
% 
%-------------------------------------------------------
function p = pairings(H)

p = length(find(H));