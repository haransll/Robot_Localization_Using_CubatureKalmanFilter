%-------------------------------------------------------
% University of Zaragoza
% Centro Politecnico Superior
% Robotics and Real Time Group
% Authors:  J. Neira, J. Tardos
% Date   :  7-2004
%-------------------------------------------------------
% slam, explore data association algorithms
%-------------------------------------------------------
clear all;
close all;
randn('state', 0);
rand('state', 0);
addpath 'tools';

% determines execution and display modes
global configuration;

configuration.ellipses = 1;
configuration.tags = 0;
configuration.odometry = 1;
configuration.noise = 1;
configuration.alpha = 0.99;
configuration.step_by_step = 0;
configuration.people = 0;

% figure numbers
configuration.ground = 1;
configuration.map = 2;
configuration.observations = 3;
configuration.compatibility = 4;
configuration.ground_hypothesis = 5;
configuration.hypothesis = 6;
configuration.tables = 7;

% variables you need in several places
global map ground sensor people chi2 results;

%chi2 = chi2inv(configuration.alpha,1:1000);

load 'data/chi2';

sensor.range = 5;
sensor.minangle = -pi/2;
sensor.maxangle = pi/2;
sensor.srho = 0.01;
sensor.stita = 0.125*pi/180;

% generate the experiment data
[ground, people] = generate_experiment;

% start with a fresh map
[map, ground] = new_map(map, ground);

% plot ground
draw_ground(ground);
pause

if configuration.people
    people.x = []; 
    people.y = [];
end

% ok, here we go
step = 1;
observations = get_observations(ground, sensor, step);
draw_observations (observations, ground, step);

GT = zeros(1, observations.m);
H = zeros(1, observations.m);

    map = add_features(map, observations);

results.total = [];
results.true.positives = [];
results.true.negatives = [];
results.false.positives = [];
results.false.negatives = [];

results = store_results (results, observations, GT, H);

% plot map
configuration.name = '';
draw_map (map, ground, step);

steps = length(ground.motion);
for step = 2 : steps,
    
    disp('--------------------------------------------------------------');
    disp(sprintf('Step: %d', step));
    
    %  EKF prediction step
    motion = ground.motion(step - 1);    
    ground = move_vehicle (ground, motion);    
    odometry = get_odometry (motion);    
    map = EKF_prediction (map, odometry);    
    
    % sense
    observations = get_observations(ground, sensor, step);
    
    % individual compatibility
    prediction = predict_observations (map, ground);
    compatibility = compute_compatibility (prediction, observations);

    disp(sprintf('Hypothesis: %d', compatibility.HS));
    disp(['IC: ' sprintf('%2d   ', compatibility.AL)]);
    disp(' ');
    
    % ground truth
    GT = ground_solution(map, observations);
    disp(['GT: ' sprintf('%2d   ', GT)]);
    disp(' ');

    % your algorithm here!
    % 1. Try NN
    % 2. Complete SINGLES and try it
    % 3. Include people and try SINGLES
    % 4. Try JCBB
    
    H = NN (prediction, observations, compatibility);

%    configuration.step_by_step = not(prod(H == GT)); % discrepance with ground truth

    draw_map (map, ground, step);
    draw_observations (observations, ground, step);
    
    
    draw_compatibility (prediction, observations, compatibility);

    disp(['H : ' sprintf('%2d   ', H)]);
    disp(['    ' sprintf('%2d   ', GT == H)]);
    disp(' ');
    
    draw_hypothesis (prediction, observations, H, 'NN:', 'b-');
    draw_hypothesis (prediction, observations, GT, 'JCBB:', 'b-');
    draw_tables (compatibility, GT, H);
    
    % update EKF step
    map = EKF_update (map, prediction, observations, H);
    

    % only new features with no neighbours
    new = find((H == 0) & (compatibility.AL == 0));
    
    if nnz(new)
       disp(['NEW: ' sprintf('%2d   ', new)]);
       map = add_features(map, observations, new);
    end

    draw_map (map, ground, step);
    results = store_results(results, observations, GT, H);
    
    if configuration.step_by_step
        wait
    else
        drawnow;
    end
    
end

show_results(map, ground, results);
