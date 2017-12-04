function plot_path(map, path)
% PLOT_PATH Visualize a path through an environment
%   PLOT_PATH(map, path) creates a figure showing a path through the
%   environment.  path is an N-by-3 matrix where each row corresponds to the
%   (x, y, z) coordinates of one point along the path.

% CMSC 828T Proj 1 Phase 1

%% START YOUR CODE HERE %%
[xDim, yDim, zDim] = size(map{1});

close all
figure;
view(3)

nrObstacles = size(map{7}, 1);
for i = 1:nrObstacles
    x0 = map{7}(i, 1);
    y0 = map{7}(i, 2);
    z0 = map{7}(i, 3);
    x1 = map{7}(i, 4);
    y1 = map{7}(i, 5);
    z1 = map{7}(i, 6);
    r = map{7}(i, 7);
    g = map{7}(i, 8);
    b = map{7}(i, 9);
    patch_args = {'FaceColor', [r/255 g/255 b/255]};
    patch('XData', [x0 x0 x1 x1], 'YData', [y0 y1 y1 y0], ...
          'ZData', [z0 z0 z0 z0], patch_args{:})
   	hold on
    patch('XData', [x0 x0 x1 x1], 'YData', [y0 y1 y1 y0], ...
          'ZData', [z1 z1 z1 z1], patch_args{:})
    patch('XData', [x0 x0 x1 x1], 'YData', [y1 y1 y1 y1], ...
          'ZData', [z0 z1 z1 z0], patch_args{:})
    patch('XData', [x0 x0 x1 x1], 'YData', [y0 y0 y0 y0], ...
          'ZData', [z0 z1 z1 z0], patch_args{:})
    patch('XData', [x1 x1 x1 x1], 'YData', [y0 y0 y1 y1], ...
          'ZData', [z0 z1 z1 z0], patch_args{:})
    patch('XData', [x0 x0 x0 x0], 'YData', [y0 y0 y1 y1], ...
          'ZData', [z0 z1 z1 z0], patch_args{:})
end

nrPoints = size(path, 1);
for i = 1:nrPoints-1
    plot3([path(i+1,1);path(i,1)], [path(i+1,2);path(i,2)], ...
          [path(i+1,3);path(i,3)], 'g')
    hold on
end

ax = gca;
ax.XGrid = 'on';
ax.YGrid = 'on';
ax.ZGrid = 'on';
ax.XLim = [map{2}(1) map{2}(xDim + 1)];
ax.YLim = [map{3}(1) map{3}(yDim + 1)];
ax.ZLim = [map{4}(1) map{4}(zDim + 1)];
%% END YOUR CODE HERE %%

end