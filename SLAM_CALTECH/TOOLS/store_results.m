function results = store_results (results, observations, GT, H),
%-------------------------------------------------------
% University of Zaragoza
% Centro Politecnico Superior
% Robotics and Real Time Group
% Authors:  J. Neira, J. Tardos
% Date   :  7-2004
%-------------------------------------------------------
%-------------------------------------------------------

total = observations.m;
true.positives = sum((H & (H == GT)));
true.negatives = sum((not (H) & (H == GT)));
false.positives = sum(H & not (H == GT));
false.negatives = sum(not (H) & not (H == GT));

results.total = [results.total total];
results.true.positives = [results.true.positives true.positives];
results.true.negatives = [results.true.negatives true.negatives];
results.false.positives = [results.false.positives false.positives];
results.false.negatives = [results.false.negatives false.negatives];

if (not(min(H == GT)))
    disp('Hypothesis not in agreement with ground truth');
    disp(sprintf('True positives: %d', true.positives));
    disp(sprintf('True negatives: %d', true.negatives));
    disp(sprintf('False positives: %d', false.positives));
    disp(sprintf('False negatives: %d', false.negatives));
    %wait;
end


