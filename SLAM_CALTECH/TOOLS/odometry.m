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
configuration.step_by_step = 1;
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

sensor.range = 2;
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
    draw_map (map, ground, step);
    
    if configuration.step_by_step
        wait
    else
        drawnow;
    end
    
end
