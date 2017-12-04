function [xtraj, ttraj, terminate_cond] = test_trajectory(start, stop, map, path, vis)
% TEST_TRAJECTORY simulates the robot from START to STOP following a PATH
% that's been planned for MAP.
% start - a 3d vector or a cell contains multiple 3d vectors
% stop  - a 3d vector or a cell contains multiple 3d vectors
% map   - map generated by your load_map
% path  - n x 3 matrix path planned by your dijkstra algorithm
% vis   - true for displaying visualization

addpath('utils');

%Controller and trajectory generator handles
controlhandle = @controller;
trajhandle    = @trajectory_generator;

% Make cell
if ~iscell(start), start = {start}; end
if ~iscell(stop),  stop  = {stop}; end
if ~iscell(path),  path  = {path} ;end

% Get nquad
nquad = length(start);

% Make column vector
for qn = 1:nquad
    start{qn} = start{qn}(:);
    stop{qn} = stop{qn}(:);
end

% Quadrotor model
params = drone250x();

%% **************************** FIGURES *****************************
% Environment figure
if nargin < 5
    vis = true;
end

fprintf('Initializing figures...\n')
if vis
    h_fig = figure('Name', 'Environment');
else
    h_fig = figure('Name', 'Environment', 'Visible', 'Off');
end
if nquad == 1
    plot_path(map, path{1});
else
    % you could modify your plot_path to handle cell input for multiple robots
end
h_3d = gca;
drawnow;
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]')
quadcolors = lines(nquad);
set(gcf,'Renderer','OpenGL')

%% *********************** INITIAL CONDITIONS ***********************
fprintf('Setting initial conditions...\n')
% Maximum time that the quadrotor is allowed to fly
time_tol = 50;          % maximum simulation time
starttime = 0;          % start of simulation in seconds
tstep     = 0.01;       % this determines the time step at which the solution is given
cstep     = 0.05;       % image capture time interval
nstep     = cstep/tstep;
time      = starttime;  % current time
max_iter  = time_tol / cstep;      % max iteration
for qn = 1:nquad
    % Get start and stop position
    x0{qn}    = init_state(start{qn}, 0);
    xtraj{qn} = zeros(max_iter*nstep, length(x0{qn}));
    ttraj{qn} = zeros(max_iter*nstep, 1);
end

% Maximum position error of the quadrotor at goal
pos_tol  = 0.05; % m
% Maximum speed of the quadrotor at goal
vel_tol  = 0.05; % m/s

x = x0;        % state

%% ************************* RUN SIMULATION *************************
fprintf('Simulation Running....\n')
for iter = 1:max_iter
    timeint = time:tstep:time+cstep;
    tic;
    % Iterate over each quad
    for qn = 1:nquad
        % Initialize quad plot
        if iter == 1
            QP{qn} = QuadPlot(qn, x0{qn}, 0.1, 0.04, quadcolors(qn,:), max_iter, h_3d);
            desired_state = trajhandle(time, qn);
            QP{qn}.UpdateQuadPlot(x{qn}, [desired_state.pos; desired_state.vel], time);
            h_title = title(sprintf('iteration: %d, time: %4.2f', iter, time));
        end

        % Run simulation
        [tsave, xsave] = ode45(@(t,s) quadEOM(t, s, qn, controlhandle, trajhandle, params), timeint, x{qn});
        x{qn} = xsave(end, :)';
        % Save to traj
        xtraj{qn}((iter-1)*nstep+1:iter*nstep,:) = xsave(1:end-1,:);
        ttraj{qn}((iter-1)*nstep+1:iter*nstep)   = tsave(1:end-1);

        % Update quad plot
        desired_state = trajhandle(time + cstep, qn);
        QP{qn}.UpdateQuadPlot(x{qn}, [desired_state.pos; desired_state.vel], time + cstep);
    end

    set(h_title, 'String', sprintf('iteration: %d, time: %4.2f', iter, time + cstep))
    time = time + cstep; % Update simulation time
    t = toc;

    % Pause to make real-time
    if (t < cstep)
        pause(cstep - t);
    end

    % Check termination criteria
    terminate_cond = terminate_check(x, time, stop, pos_tol, vel_tol, time_tol);
    if terminate_cond
        break
    end

end

fprintf('Simulation Finished....\n')

%% ************************* POST PROCESSING *************************
% Truncate xtraj and ttraj
for qn = 1:nquad
    xtraj{qn} = xtraj{qn}(1:iter*nstep,:);
    ttraj{qn} = ttraj{qn}(1:iter*nstep);
end

% Plot the saved position and velocity of each robot
if vis
    for qn = 1:nquad
        % Truncate saved variables
        QP{qn}.TruncateHist();
        % Plot position for each quad
        h_pos{qn} = figure('Name', ['Quad ' num2str(qn) ' : position']);
        plot_state(h_pos{qn}, QP{qn}.state_hist(1:3,:), QP{qn}.time_hist, 'pos', 'vic');
        plot_state(h_pos{qn}, QP{qn}.state_des_hist(1:3,:), QP{qn}.time_hist, 'pos', 'des');
        % Plot velocity for each quad
        h_vel{qn} = figure('Name', ['Quad ' num2str(qn) ' : velocity']);
        plot_state(h_vel{qn}, QP{qn}.state_hist(4:6,:), QP{qn}.time_hist, 'vel', 'vic');
        plot_state(h_vel{qn}, QP{qn}.state_des_hist(4:6,:), QP{qn}.time_hist, 'vel', 'des');
    end
    
    actualTraj = QP{1}.state_hist(1:3, :)';
    sum(collide_v2(map, actualTraj))
    nrPoints = size(actualTraj, 1);
    path_length = 0;
    for i = 1:nrPoints-1
        init = actualTraj(i, :);
        final = actualTraj(i + 1, :);
        
        dx = final(1) - init(1);
        dy = final(2) - init(2);
        dz = final(3) - init(3);
        dist = sqrt(dx^2 + dy^2 + dz^2);
        path_length = path_length + dist;
    end
    path_length
end

end
