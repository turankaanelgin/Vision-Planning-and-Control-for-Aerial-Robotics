function [F, M, trpy, drpy] = controller(qd, t, qn, params)
% CONTROLLER quadrotor controller
% The current states are:
% qd{qn}.pos, qd{qn}.vel, qd{qn}.euler = [roll;pitch;yaw], qd{qn}.omega
% The desired states are:
% qd{qn}.pos_des, qd{qn}.vel_des, qd{qn}.acc_des, qd{qn}.yaw_des, qd{qn}.yawdot_des
% Using these current and desired states, you have to compute the desired controls

% =================== Your code goes here ===================

% Desired roll, pitch and yaw
ep = qd{qn}.pos_des - qd{qn}.pos;
ev = qd{qn}.vel_des - qd{qn}.vel;
acc_des = params.k(1,:)' .* ep + params.k(2,:)' .* ev + qd{qn}.acc_des;

phi_des = 1.0 / params.grav * (acc_des(1) * sin(qd{qn}.yaw_des) - ...
                               acc_des(2) * cos(qd{qn}.yaw_des));
theta_des = 1.0 / params.grav * (acc_des(1) * cos(qd{qn}.yaw_des) + ...
                                 acc_des(2) * sin(qd{qn}.yaw_des));
psi_des = qd{qn}.yaw_des;

if phi_des > params.maxangle 
    phi_des = params.maxangle;
end
if theta_des > params.maxangle
    theta_des = params.maxangle;
end
if psi_des > params.maxangle 
    psi_des = params.maxangle;
end

% Thrust
F    = params.mass * (params.grav + acc_des(3));
if F < params.minF
    F = params.minF;
elseif F > params.maxF
    F = params.maxF;
end

% Moment
M    = zeros(3,1); % You should fill this in
M(1) = params.k_angle(1, 1) * (phi_des - qd{qn}.euler(1)) + ...
       params.k_angle(2, 1) * (0 - qd{qn}.omega(1));
M(2) = params.k_angle(1, 2) * (theta_des - qd{qn}.euler(2)) + ...
       params.k_angle(2, 2) * (0 - qd{qn}.omega(2));
M(3) = params.k_angle(1, 3) * (psi_des - qd{qn}.euler(3)) + ...
       params.k_angle(2, 3) * (qd{qn}.yawdot_des - qd{qn}.omega(3));
% =================== Your code ends here ===================

% Output trpy and drpy as in hardware
trpy = [F, phi_des, theta_des, psi_des];
drpy = [0, 0,       0,         0];

end
