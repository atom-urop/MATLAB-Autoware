function nxt = nextStates(costmap, vehicle, x, y, th)
%NEXTSTATES  The six moves available from one state.
%
% Reproduces the move set of AstarSearch::expandNodes  astar_search.cpp:776
% Parameter values from config/freespace_planner.param.yaml
%
% nxt : [n x 4] rows of [x y theta is_back] — collision-free moves only

max_turning_ratio  = 0.5;      % yaml
turning_steps      = 1;        % yaml
expansion_distance = 0.5;      % yaml  [m]

L         = vehicle.param.wheel_base;
max_steer = vehicle.param.max_steer_angle * max_turning_ratio;
steer_res = max_steer / turning_steps;

nxt = zeros(0,5);
for is_back = [false true]
    d = expansion_distance;
    if is_back, d = -d; end
    for si = -turning_steps : turning_steps
        [x2, y2, th2] = bicycleGetPose(x, y, th, si*steer_res, L, d);
        if ~collisionCheck(costmap, vehicle, x2, y2, th2)
            nxt(end+1,:) = [x2 y2 th2 double(is_back) si];    %#ok<AGROW>
        end
    end
end
end