function H = NN (prediction, observations, compatibility)
%-------------------------------------------------------
%-------------------------------------------------------
global chi2;
global configuration;

for i = 1:observations.m,
    D2min = compatibility.d2 (i, 1);
    nearest = 1;
    for j = 2:prediction.n,
        Dij2 = compatibility.d2 (i, j);
        if Dij2 < D2min
            nearest = j;
            D2min = Dij2;
        end
    end
    if D2min <= chi2(2)
        H(i) = nearest;
    else
        H(i) = 0;
    end    
end

configuration.name = 'NEAREST NEIGHBOUR';
