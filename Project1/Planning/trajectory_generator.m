function [ desired_state ] = trajectory_generator(t, qn, map, path)
% TRAJECTORY_GENERATOR: Turn a Dijkstra or A* path into a trajectory
%
% NOTE: This function would be called with variable number of input
% arguments. In init_script, it will be called with arguments
% trajectory_generator([], [], map, path) and later, in test_trajectory,
% it will be called with only t and qn as arguments, so your code should
% be able to handle that. This can be done by checking the number of
% arguments to the function using the "nargin" variable, check the
% MATLAB documentation for more information.
%
% map: The map structure returned by your load_map function
% path: This is the path returned by your planner (dijkstra function)
%
% desired_state: Contains all the information that is passed to the
% controller, as in phase 2
%
% It is suggested to use "persistent" variables to store map and path
% during the initialization call of trajectory_generator, e.g.
persistent map0 path0
persistent polynomial
persistent totalTime
persistent nrPoints
persistent lastPos
persistent path_length
persistent bangTime

if nargin == 2
    if nrPoints > 0
        interval = nrPoints;
        time = 0;
        startTime = 0;
        
        % According to t parameter, compute which interval
        % it belongs to
        for i = 1:nrPoints - 1
            start = path0(i, :);
            finish = path0(i + 1, :);
            dx = start(1) - finish(1);
            dy = start(2) - finish(2);
            dz = start(3) - finish(3);
            dist = sqrt(dx^2 + dy^2 + dz^2);
            ratio = dist / path_length;
            % small times cause overshooting, that's why there is a lower
            % limit
            unitTime = max(ratio * totalTime, 1.0); % time of current interval
            time = time + unitTime;

            if time >= t % choose the interval and start time
                interval = i;
                startTime = time - unitTime;
                break;
            end
        end

        if interval <= nrPoints-1
%           BANG-COAST-BANG
            start = path0(interval, :);
            finish = path0(interval + 1, :);
            dx = finish(1) - start(1);
            dy = finish(2) - start(2);
            dz = finish(3) - start(3);
            dist = sqrt(dx^2 + dy^2 + dz^2);
            ratio = dist / path_length;
            unitTime = max(ratio * totalTime, 1.0);
            t1 = startTime + bangTime*unitTime;
            t2 = startTime + (1-bangTime)*unitTime;
            % determine which phase of bang-coast-bang to execute
            if t <= t1 
                index = 1;
            elseif t1 < t && t <= t2
                index = 2;
            else
                index = 3;
            end

            coeff = polynomial{interval, index};
            if index == 2
                b0 = coeff(1, :);
                b1 = coeff(2, :);
                pos = b0' + b1'*t;
                vel = b1';
                acc = [0; 0; 0];
            else
                c0 = coeff(1, :);
                c1 = coeff(2, :);
                c2 = coeff(3, :);

                pos = c0' + c1'*t + c2'*t^2;
                vel = c1' + 2*c2'*t;
                acc = 2*c2';
            end 
            
%             coeff = polynomial{interval};
            
% %             if unitTime < 0.2
% %                 b0 = coeff(1, :);
% %                 b1 = coeff(2, :);
% %                 pos = b0' + b1'*t;
% %                 vel = b1';
% %                 acc = [0; 0; 0];
% %             else
%                 c0 = coeff(1, :);
%                 c1 = coeff(2, :);
%                 c2 = coeff(3, :);
%                 c3 = coeff(4, :);
%                 c4 = coeff(5, :);
%                 c5 = coeff(6, :);
%                 pos = c0' + c1'*t + c2'*t^2 + c3'*t^3 + c4'*t^4 + c5'*t^5;
%                 vel = c1' + 2*c2'*t + 3*c3'*t^2 + 4*c4'*t^3 + 5*c5'*t^4;
%                 acc = 2*c2' + 6*c3'*t + 12*c4'*t^2 + 20*c5'*t^3;
% %             end
            lastPos = pos(:); % save the last position in case it stops later
        else
            pos = lastPos(:);
            vel = [0; 0; 0];
            acc = [0; 0; 0];
        end 
    else
        pos = [0; 0; 0];
        vel = [0; 0; 0];
        acc = [0; 0; 0];
    end
    yaw = 0;
    yawdot = 0;
elseif nargin == 4
    map0 = map;
    path0 = path{1};
    path0 = simplify_path(map, path0);
    
    nrPoints = size(path0, 1);
    if nrPoints > 0
        polynomial = cell(nrPoints-1, 1);
        bangTime = 0.4;
        % Compute the total path length
        path_length = 0;
        for i = 1:nrPoints-1
            start = path0(i, :);
            finish = path0(i + 1, :);
            dx = start(1) - finish(1);
            dy = start(2) - finish(2);
            dz = start(3) - finish(3);
            dist = sqrt(dx^2 + dy^2 + dz^2);
            path_length = path_length + dist;
        end
        totalTime = 0.6 * path_length;

        time = 0;
        prevVel = [0 0 0]; % initial velocity of a segment should be equal 
                           % to the final velocity of the previous one 
        
        for i = 1:nrPoints-1
            start = path0(i, :);
            finish = path0(i + 1, :);
            dx = finish(1) - start(1);
            dy = finish(2) - start(2);
            dz = finish(3) - start(3);
            dist = sqrt(dx^2 + dy^2 + dz^2);
            ratio = dist / path_length;
            unitTime = max(ratio * totalTime, 1.0); % time of the current segment
            t1 = time + bangTime*unitTime;
            t2 = time + (1-bangTime)*unitTime;

            v = [dx dy dz] / (t2 - time);
            a = v / (t1 - time);
            ftime = time + unitTime; % final time
            
%             B = [1, time, time^2, time^3, time^4, time^5;
%                  1, ftime, ftime^2, ftime^3, ftime^4, ftime^5;
%                  0, 1, 2*time, 3*time^2, 4*time^3, 5*time^4;
%                  0, 1, 2*ftime, 3*ftime^2, 4*ftime^3, 5*ftime^4;
%                  0, 0, 2, 6*time, 12*time^2, 20*time^3;
%                  0, 0, 2, 6*ftime, 12*ftime^2, 20*ftime^3];
%             
%             v_av = 0.1 * [dx dy dz] / dist; % average velocity in that segment
%             A = zeros(6, 3);
%             A(1, :) = start(:);
%             A(2, :) = finish(:);
%             A(3, :) = vel_profile(i, :);
%             A(4, :) = vel_profile(i + 1, :);
%             % if it is the last segment, final velocity must be 0;
%             % otherwise adjust it according to the average velocity;
%             % limit with max and min values to prevent going too fast
%             
%           
%             % compute the desired acceleration accordingly
%             A(5, :) = [0 0 0];
%             A(6, :) = [0 0 0];
%             polynomial{i} = B \ A;
%           BANG-COAST-BANG 
            
            A = [start; start + 0.5*a*(t1-time)^2; [0 0 0]; v; a; a];
            B = [1, time, time^2; 1, t1, t1^2;
                 0, 1, 2*time; 0, 1, 2*t1;
                 0, 0, 2; 0, 0, 2];
            coeff1 = B \ A;
            A = [start + 0.5*a*(t1-time)^2; start + 0.5*a*(t1-time)^2 + v*(t2-t1); 
                 v; v];
            B = [1, t1; 1, t2; 0, 1; 0, 1];
            coeff2 = B \ A;
            A = [start + 0.5*a*(t1-time)^2 + v*(t2-t1); finish; 
                 v; [0 0 0]; -a; -a];
            B = [1, t2, t2^2; 1, ftime, ftime^2;
                 0, 1, 2*t2; 0, 1, 2*ftime;
                 0, 0, 2; 0, 0, 2];
            coeff3 = B \ A;
            polynomial{i, 1} = coeff1;
            polynomial{i, 2} = coeff2;
            polynomial{i, 3} = coeff3;
            time = ftime;
        end
    end
    
    pos = [0; 0; 0];
    vel = [0; 0; 0];
    acc = [0; 0; 0];
    yaw = 0;
    yawdot = 0;
end

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;