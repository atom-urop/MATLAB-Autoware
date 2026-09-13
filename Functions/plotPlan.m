function plotPlan(costmap, vehicle, path, step)
%PLOTPLAN  Draw the planned parking maneuver as a sequence of vehicle rectangles.
%
%   plotPlan(costmap, vehicle, path)      draws every planned pose
%   plotPlan(costmap, vehicle, path, 2)   draws every 2nd pose
%
%   Uses figure 99 exclusively, so no Simulink animation block can draw
%   into it. blue = forward, magenta = reversing,
%   green = start pose, red = final parked pose.
%
%   The rectangle is the TRUE vehicle body. The planner checks collisions
%   against a body inflated by vehicle_shape_margin_m = 0.5 m, so real
%   clearance is larger than it looks here.

if nargin < 4, step = 1; end
p = vehicle.param;

% --- true vehicle body, from base_link (rear axle centre) ---------------
len   = p.front_overhang + p.wheel_base + p.rear_overhang;
wid   = p.wheel_tread + p.left_overhang + p.right_overhang;
back  = -p.rear_overhang;
front =  len + back;
body  = [front front back  back  front ;
         wid/2 -wid/2 -wid/2 wid/2 wid/2];

% --- our own figure, cleared each time ----------------------------------
hFig = figure(99);
set(hFig, 'Name', 'parking plan', 'NumberTitle', 'off');
clf(hFig);
ax = axes(hFig);
hold(ax,'on');

% --- the costmap --------------------------------------------------------
res = double(costmap.msg.info.resolution);
W   = double(costmap.msg.info.width);
H   = double(costmap.msg.info.height);
ox  = double(costmap.msg.info.origin.position.x);
oy  = double(costmap.msg.info.origin.position.y);
g   = double(costmap.msg.data);

imagesc(ax, ox+(0:W-1)*res, oy+(0:H-1)*res, g, 'AlphaData', (g>=100)*0.55);
colormap(ax, [1 1 1; 0.85 0.25 0.25]);
set(ax, 'YDir','normal', 'Layer','top');

% --- a rectangle at every planned pose ----------------------------------
idx = 1:step:size(path,1);
if idx(end) ~= size(path,1), idx(end+1) = size(path,1); end

for k = idx
    th = path(k,3);
    R  = [cos(th) -sin(th); sin(th) cos(th)];
    B  = R*body + path(k,1:2)';
    if path(k,4) == 1
        col = [0.85 0.20 0.75];        % reversing
    else
        col = [0.10 0.45 0.90];        % forward
    end
    plot(ax, B(1,:), B(2,:), '-', 'Color', col, 'LineWidth', 1.0);
    plot(ax, [path(k,1), path(k,1)+1.2*cos(th)], ...
             [path(k,2), path(k,2)+1.2*sin(th)], '-', 'Color', col, 'LineWidth', 0.8);
end

plot(ax, path(:,1), path(:,2), 'k-', 'LineWidth', 1.2);

drawBody(path(1,1:3),   [0.10 0.70 0.20]);   % start
drawBody(path(end,1:3), [0.90 0.15 0.15]);   % parked

sw = find(diff(path(:,4)) ~= 0);             % gear changes
plot(ax, path(sw,1), path(sw,2), 'ks', 'MarkerSize', 9, 'LineWidth', 1.5);

axis(ax,'equal'); grid(ax,'on');
xlim(ax,[-16 16]); ylim(ax,[-14 14]);
xlabel(ax,'x [m]'); ylabel(ax,'y [m]');
title(ax, sprintf('planned maneuver:  %.2f m,  %d gear reversals,  %d poses', ...
      sum(hypot(diff(path(:,1)), diff(path(:,2)))), numel(sw), size(path,1)));
drawnow;

    function drawBody(q, col)
        Rr = [cos(q(3)) -sin(q(3)); sin(q(3)) cos(q(3))];
        Bb = Rr*body + q(1:2)';
        fill(ax, Bb(1,:), Bb(2,:), col, 'FaceAlpha', 0.35, ...
             'EdgeColor', col, 'LineWidth', 2);
    end
end