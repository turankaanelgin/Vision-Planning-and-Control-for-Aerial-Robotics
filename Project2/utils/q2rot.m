function R = q2rot(qx, qy, qz, qw)
    R = zeros(3, 3);
    R(1, 1) = 1 - 2*qy^2 - 2*qz^2;
    R(1, 2) = 2*qx*qy + 2*qw*qz;
    R(1, 3) = 2*qx*qz - 2*qw*qy;
    R(2, 1) = 2*qx*qy - 2*qw*qz;
    R(2, 2) = 1 - 2*qx^2 - 2*qz^2;
    R(2, 3) = 2*qy*qz + 2*qw*qx;
    R(3, 1) = 2*qx*qz + 2*qw*qy;
    R(3, 2) = 2*qy*qz - 2*qw*qx;
    R(3, 3) = 1 - 2*qx^2 - 2*qy^2;
end