function C = verify_path(map, path)
    nrPoints = size(path, 1);
    xy_res = map{5};
    z_res = map{6};
    C = 0;
    for i = 1:nrPoints-1
        init = path(i, :);
        final = path(i+1, :);
        
        xPoints = init(1):xy_res:final(1);
        yPoints = init(2):xy_res:final(2);
        zPoints = init(3):z_res:final(3);
        
        length = max(max(size(xPoints, 2), size(yPoints, 2)), size(zPoints, 2));
        xPoints = linspace(init(1), final(1), length);
        yPoints = linspace(init(2), final(2), length);
        zPoints = linspace(init(3), final(3), length);
        set_of_points = zeros(length, 3);
        for k = 1:length
            set_of_points(k, :) = [xPoints(1, k) yPoints(1, k) zPoints(1, k)];
        end
        
        A = collide(map, set_of_points);
        nrCollisions = sum(A(:));
        if nrCollisions ~= 0
            C = 1;
        end
    end
end