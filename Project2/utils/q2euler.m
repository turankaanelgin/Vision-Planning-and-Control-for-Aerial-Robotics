function [pitch, roll] = q2euler(qx, qy, qz, qw)
    sinr = 2.0 * (qw * qx + qy * qz);
    cosr = 1 - 2 * (qx * qx + qy * qy);
    roll = atan2(sinr, cosr);
    
    sinp = 2.0 * (qw * qy - qz * qx);
    if abs(sinp) >= 1
        pitch = asin(pi/2);
    else
        pitch = asin(sinp);
    end
end