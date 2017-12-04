close all;
clear all;
clc;

%% Plan path
disp('Planning ...');
% map = load_map('maps/map3.txt', 0.1, 0.5, 0.25);
% start = {[1 3 5]};
% stop  = {[19 3 5]};

% map = load_map('maps/map3.txt', 0.1, 1.0, 0.2);
% start = {[2.0 0.1 0.1]};
% stop  = {[14.0 2.0 5.7]};

% map = load_map('maps/map1.txt', 0.1, 1.0, 0.2);
% start = {[5.0 -4.9 3.0]};
% stop  = {[9.8 19.8 5.8]};

% map = load_map('maps/map1.txt', 0.1, 1.0, 0.2);
% start = {[0.0 -4.9 0.2]};
% stop  = {[8.0 18.0 2.1]};
% 
map = load_map('maps/map2.txt', 0.2, 0.2, 0.2);
start = {[0.5 20 2]};
stop  = {[9.5 10.0 4]};

nquad = length(start);
for qn = 1:nquad
    path{qn} = dijkstra(map, start{qn}, stop{qn}, true);
end
if nquad == 1
    plot_path(map, path{1});
else
    % you could modify your plot_path to handle cell input for multiple robots
end

%% Additional init script
init_script;

%% Run trajectory
trajectory = test_trajectory(start, stop, map, path, true); % with visualization
