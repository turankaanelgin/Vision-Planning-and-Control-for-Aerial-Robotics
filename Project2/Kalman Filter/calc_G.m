function G = calc_G(phi, theta)
    % Calculates G matrix used in Kalman filter
    G = [cos(theta), 0, -cos(phi)*sin(theta); 
         0, 1, sin(phi);
         sin(theta), 0, cos(phi)*cos(theta)];
end