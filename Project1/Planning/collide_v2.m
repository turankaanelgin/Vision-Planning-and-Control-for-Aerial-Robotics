function C = collide_v2(map, points)
    nrPoints = size(points, 1);
    C = zeros(nrPoints, 1);
    
    for i = 1:nrPoints
        x = points(i, 1);
        y = points(i, 2); 
        z = points(i, 3);
        
        obstacles = map{7};
        nrObstacles = size(obstacles, 1);
        for j = 1:nrObstacles
            obs = obstacles(j, :);
            if obs(1) - 0.15 <= x && x <= obs(4) + 0.15 && ...
               obs(2) - 0.15 <= y && y <= obs(5) + 0.15 && ...
               obs(3) - 0.1 <= z && z <= obs(6) + 0.1
                C(i) = 1;
            end
        end
    end
end