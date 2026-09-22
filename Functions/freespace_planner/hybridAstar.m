%==========================================================================
%% 4WS with new approach (new cost function) : added a term for the steering rate 
% + added costs around obstacles
% + added new cases for the heuristic function
function path = hybridAstar(costmap, vehicle, start_pose, goal_pose)
%HYBRIDASTAR  The search loop.
%
% Reproduces, from autoware_freespace_planning_algorithms:
%   AstarSearch::search       astar_search.cpp:668
%   AstarSearch::expandNodes  astar_search.cpp:776
%   AstarSearch::isGoal       astar_search.cpp:1108
%   AstarSearch::setPath      astar_search.cpp:1000
%
% DEVIATION: the heuristic is plain Euclidean distance. Autoware uses
%   max(Dijkstra-from-goal, ReedsShepp)  astar_search.cpp:620
% Ours is still optimistic, just weaker, so the search explores more nodes.

theta_size = 120;                      % yaml



%% WEIGHTS
curve_w    = 0.5;   
reverse_w = 0.7;   % curve_weight, reverse_weight
dir_w      = 2.0;   
heur_w    = 1.2; %1.2   % direction_change_weight, distance_heuristic_weight
steer_change_w = 0.01; % The weight used for the steering rate 
clearance_w = 0.5;%0.5          % Experimental starting weight to an obstacle
clearance_preferred = 0.6;  % Preferred additional clearance to an obstacle [m]


lon_r = 0.25;
lat_r = 0.15;
ang_r = deg2rad(2); %goal pose tolerance

%% Heuristics
heuristic_type = 1;
% 1 = Euclidean
% 2 = position + orientation lower bound

%% Parameters to tune!!
max_turning_ratio = 0.8;

delta_max = vehicle.param.max_steer_angle * max_turning_ratio;
% Maximum curvature for symmetric 4WS counter-phase:
% front = +delta_max, rear = -delta_max
kappa_max = 2*sin(delta_max) / vehicle.param.wheel_base;


turning_steps     = 5;
step  = 0.5;                           % expansion_distance

max_iter = 1000000;



res = costmap.res;  ox = costmap.origin(1);  oy = costmap.origin(2);
W = double(costmap.width);  H = double(costmap.height);
% Calculate the obstacle-distance map once per search.
grid_heuristic = [];

if heuristic_type == 3
    grid_heuristic = buildDijkstraHeuristic(costmap, goal_pose);
end

CAP    = 200000;
nodes  = zeros(CAP,11);        % x y th g f is_back si dir_dist parent
closed = false(CAP,1);

seen   = zeros(W*H*theta_size, 1, 'int32');    % grid key -> node row

% Sparse storage: state key -> row in nodes
% seen = containers.Map( ...
%     'KeyType', 'uint64', ...
%     'ValueType', 'int32');

openF  = zeros(CAP,1);  openI = zeros(CAP,1);  on = 0;

nodes(1,:) = [start_pose(1) start_pose(2) start_pose(3) 0 0 0 0 0 0 0 0];

% nodes(1,5) = heur_w*hypot(start_pose(1)-goal_pose(1), start_pose(2)-goal_pose(2));
nodes(1,5) = heur_w * heuristicCost( ...
    start_pose(1), start_pose(2), start_pose(3)); %new heuristic approach

seen(k3(start_pose(1),start_pose(2),start_pose(3))) = 1;

% start_key = stateKey( ...
%     start_pose(1), start_pose(2), start_pose(3), ...
%     0, 0, false);
% 
% seen(start_key) = int32(1);

nn = 1;  on = 1;  openF(1) = nodes(1,5);  openI(1) = 1;

found = 0;
for iter = 1:max_iter
    if on == 0, break; end
    [~,k] = min(openF(1:on));                       % take the most promising
    cur = openI(k);
    openF(k) = openF(on);  openI(k) = openI(on);  on = on - 1;

    if closed(cur), continue; end
    closed(cur) = true;
    if atGoal(cur), found = cur; break; end

    nxt = nextStates( ...
    costmap, vehicle, ...
    nodes(cur,1), nodes(cur,2), nodes(cur,3), ...
    max_turning_ratio, turning_steps, step);

    for j = 1:size(nxt,1)
        nx = nxt(j,1);  
        ny = nxt(j,2);  
        nth = nxt(j,3);
        is_back = nxt(j,4); 
        front_index = nxt(j,5);
        rear_index = nxt(j,6);
        mode = nxt(j,7);

        kk = k3(nx,ny,nth);  
        r = seen(kk);
        if r > 0 && closed(r), continue; end
        % kk = stateKey( ...
        %     nx, ny, nth, ...
        %     front_index, rear_index, is_back);
        % 
        % if isKey(seen, kk)
        %     r = double(seen(kk));
        % else
        %     r = 0;
        % end
        % 
        % if r > 0 && closed(r)
        %     continue;
        % end
        % 
        switched = (nodes(cur,11) ~= 0) && (is_back ~= nodes(cur,6));

        %% Steering magnitude cost

        normalized_steering = ...
            max(abs(front_index),abs(rear_index)) / turning_steps;

        w = 1 + curve_w*normalized_steering;

        if is_back
            w = w*(1 + reverse_w);
        end

        %% Steering-change cost

        % Steering indices used to reach the current node
        previous_front_index = nodes(cur,7);
        previous_rear_index  = nodes(cur,8); %This is for the purpose of adding the rate of change of steering into the cost function

        % Normalized change between the previous and candidate commands
        front_change = ...
            abs(front_index - previous_front_index) / turning_steps;

        rear_change = ...
            abs(rear_index - previous_rear_index) / turning_steps;

        %% Quadratic cost strongly penalizes sharp steering changes
        steering_change_cost = steer_change_w * ...
            (front_change^2 + rear_change^2);

        %% Obstacle-clearance cost
        clearance_cost = 0;

        if clearance_w > 0
            steer_res = vehicle.param.max_steer_angle * ...
                max_turning_ratio / turning_steps;

            steer_f = front_index * steer_res;
            steer_r = rear_index  * steer_res;

            distance = step;
            if is_back
                distance = -distance;
            end

            % Sample the actual bicycle-model movement, not a straight chord
            n_samples = max(2, ceil(step / (costmap.res/2)));
            d_min = clearance_preferred;

            for s = 0:n_samples
                [xs, ys, ths] = bicycleGetPose( ...
                    nodes(cur,1), nodes(cur,2), nodes(cur,3), ...
                    steer_f, steer_r, vehicle.param.wheel_base, ...
                    distance * s/n_samples);

                d = vehicleClearance( ...
                    costmap, vehicle, xs, ys, ths, clearance_preferred);

                d_min = min(d_min, d);
            end

            clearance_cost = clearance_w * step * ...
                max(0, 1 - d_min/clearance_preferred)^2;
        end

        %% Complete accumulated cost

        g = nodes(cur,4) + ...
            w*step + ...
            steering_change_cost + ...
            clearance_cost;

        if switched
            g = g + dir_w*(1 + 1/(1 + nodes(cur,10)));
        end



        % f = g + heur_w*hypot(nx-goal_pose(1), ny-goal_pose(2)); %%The complete cost computation when adding the cummulative g cost with the heuristic cost that is f = g + 2*h
        f = g + heur_w*heuristicCost(nx, ny, nth); %%Computation of the Hybrid A* cost function considering the different approaches of the heuristic function

        if r == 0 || f < nodes(r,5)
            if r == 0, nn = nn + 1;  r = nn;  seen(kk) = int32(r); end
            dd = step;  if ~switched, dd = dd + nodes(cur,10); end
            nodes(r,:) = [ ...
                            nx, ny, nth, g, f, double(is_back), ...
                            front_index, rear_index, mode, dd, cur];
            on = on + 1;  openF(on) = f;  openI(on) = r;
        end
    end
end

if found
    ch = found;
    while nodes(ch(end),11) ~= 0, ch(end+1) = nodes(ch(end),11); end   %#ok<AGROW>
    %path = nodes(flip(ch), [1 2 3 6]);
    path = nodes(flip(ch), [1 2 3 6 7 8 9]);

    % Columns 5 and 6 are steering INDICES. Convert them to radians.
    steer_res = vehicle.param.max_steer_angle * max_turning_ratio / turning_steps;
    path(:,5) = path(:,5) * steer_res;      % front wheel angle [rad]
    path(:,6) = path(:,6) * steer_res;      % rear  wheel angle [rad]

    fprintf('plan found: %.2f m, %d reversals, %d iterations\n', ...
        sum(hypot(diff(path(:,1)),diff(path(:,2)))), nnz(diff(path(:,4))~=0), iter);
else
    % path = zeros(0,4);
    path = zeros(0,7);
    fprintf('no plan after %d iterations\n', iter);
end


%% Functions
%Heuristic function cost computation according to different approaches
    function h = heuristicCost(x, y, theta)

        % Position error
        position_distance = hypot( ...
            x-goal_pose(1), ...
            y-goal_pose(2));

        switch heuristic_type

            case 1
                % Original Euclidean heuristic
                h = position_distance;

            case 2
                % Wrapped orientation error in [0, pi]
                angle_error = abs(atan2( ...
                    sin(theta-goal_pose(3)), ...
                    cos(theta-goal_pose(3))));

                % Minimum distance needed to correct orientation
                orientation_distance = angle_error / kappa_max;

                h = max(position_distance, orientation_distance);
            case 3
                % Default when a grid estimate is unavailable.
                h = position_distance;

                % Convert the candidate position into map indices.
                ix = round((x - ox)/res) + 1;
                iy = round((y - oy)/res) + 1;

                if ix >= 1 && ix <= W && iy >= 1 && iy <= H
                    obstacle_distance = grid_heuristic(iy, ix);

                    if isfinite(obstacle_distance)
                        h = max(position_distance, obstacle_distance);
                    end
                end

            otherwise
                h = position_distance;
        end
    end

%%Dijkstra's function used for the heuristic function in case 3
    function hmap = buildDijkstraHeuristic(costmap, goal_pose)
        %BUILDDIJKSTRAHEURISTIC  Eight-neighbour Dijkstra distance to the goal [m].
        % Call once per planning request, then look up hmap(row, column) at each node.
        % costmap.grid uses the planner's 0..1 values (unknown < 0), NOT bus 0..100.
        % This is a point-robot grid estimate; footprint, heading and steering costs
        % remain the responsibility of Hybrid A*. It is not a proven lower bound
        % for continuous vehicle motion, because grid directions/positions are finite.
        % Inf means occupied, disconnected, or an unavailable goal cell. The caller
        % should fall back to its Euclidean estimate rather than prune those nodes.
        % MATLAB graph/distances run in the existing extrinsic planner function.

        res = double(costmap.res);
        W = double(costmap.width);
        H = double(costmap.height);
        origin = double(costmap.origin);

        validateattributes(res, {'double'}, {'scalar','finite','positive'});
        validateattributes(W, {'double'}, {'scalar','finite','integer','positive'});
        validateattributes(H, {'double'}, {'scalar','finite','integer','positive'});
        validateattributes(origin, {'double'}, {'vector','numel',2,'finite'});
        validateattributes(goal_pose, {'numeric'}, {'vector','numel',3,'real','finite'});
        assert(numel(costmap.grid) == H*W, ...
            'buildDijkstraHeuristic:GridSize', 'Grid size does not match height/width.');

        grid = reshape(double(costmap.grid), H, W);
        free = isfinite(grid) & grid >= 0 & grid < 1;
        hmap = inf(H,W);

        % Match Hybrid A*'s world-to-cell convention (MATLAB indexing adds one).
        gx = round((double(goal_pose(1))-origin(1))/res) + 1;
        gy = round((double(goal_pose(2))-origin(2))/res) + 1;
        if gx < 1 || gx > W || gy < 1 || gy > H
            warning('buildDijkstraHeuristic:GoalOutside', ...
                'Goal cell is outside the grid; use the Euclidean fallback.');
            return;
        end
        if ~free(gy,gx)
            warning('buildDijkstraHeuristic:GoalBlocked', ...
                'Goal cell is occupied/unknown; use the Euclidean fallback.');
            return;
        end

        node_id = reshape(1:H*W, H, W);
        % Four directions are enough for an undirected graph (eight neighbours).
        offsets = [1 0; 0 1; 1 1; 1 -1]; % [column offset, row offset]
        sources = cell(4,1);
        targets = cell(4,1);
        weights = cell(4,1);

        for k = 1:4
            dc = offsets(k,1);
            dr = offsets(k,2);
            rows = max(1,1-dr):min(H,H-dr);
            cols = max(1,1-dc):min(W,W-dc);

            valid = free(rows,cols) & free(rows+dr,cols+dc);
            if dc ~= 0 && dr ~= 0
                % Do not connect diagonally through the corner of an occupied cell.
                valid = valid & free(rows,cols+dc) & free(rows+dr,cols);
            end

            from = node_id(rows,cols);
            to = node_id(rows+dr,cols+dc);
            sources{k} = from(valid);
            targets{k} = to(valid);
            sources{k} = sources{k}(:);
            targets{k} = targets{k}(:);
            weights{k} = res*hypot(dc,dr)*ones(numel(sources{k}),1);
        end

        G = graph(vertcat(sources{:}), vertcat(targets{:}), ...
            vertcat(weights{:}), H*W);
        goal_id = node_id(gy,gx);
        % Supply ONE source. Omitting goal_id would compute a huge all-pairs matrix.
        d = distances(G, goal_id, 'Method', 'positive');
        hmap = reshape(d, H, W);
    end


    function kk = k3(x,y,th)
        ix = round((x-ox)/res);   iy = round((y-oy)/res);
        it = mod(round(mod(th,2*pi)/(2*pi/theta_size)), theta_size);
        kk = it*(W*H) + iy*W + ix + 1;
    end
    % function kk = stateKey( ...
    %         x, y, th, front_index, rear_index, is_back)
    % 
    %     % Discretized pose
    %     ix = round((x-ox)/res);
    %     iy = round((y-oy)/res);
    % 
    %     it = mod( ...
    %         round(mod(th,2*pi)/(2*pi/theta_size)), ...
    %         theta_size);
    % 
    %     % Zero-based pose identifier
    %     pose_key = ...
    %         uint64(it) * uint64(W*H) + ...
    %         uint64(iy) * uint64(W) + ...
    %         uint64(ix);
    % 
    %     % Steering indices range from -turning_steps to +turning_steps
    %     number_of_steering_values = uint64(2*turning_steps + 1);
    % 
    %     front_key = uint64(front_index + turning_steps);
    %     rear_key  = uint64(rear_index  + turning_steps);
    %     direction_key = uint64(logical(is_back));
    % 
    %     % Unique key for:
    %     % pose + front steering + rear steering + direction
    %     kk = pose_key;
    % 
    %     kk = kk * number_of_steering_values + front_key;
    %     kk = kk * number_of_steering_values + rear_key;
    %     kk = kk * uint64(2) + direction_key;
    % 
    %     % Keep key strictly positive
    %     kk = kk + uint64(1);
    % end

    function tf = atGoal(r)
        dx =  cos(goal_pose(3))*(nodes(r,1)-goal_pose(1)) + sin(goal_pose(3))*(nodes(r,2)-goal_pose(2));
        dy = -sin(goal_pose(3))*(nodes(r,1)-goal_pose(1)) + cos(goal_pose(3))*(nodes(r,2)-goal_pose(2));
        dt = atan2(sin(nodes(r,3)-goal_pose(3)), cos(nodes(r,3)-goal_pose(3)));
        tf = abs(dx) <= lon_r && abs(dy) <= lat_r && abs(dt) <= ang_r;
    end
end




% %==========================================================================
% %% 4WS 
% function path = hybridAstar(costmap, vehicle, start_pose, goal_pose)
% %HYBRIDASTAR  The search loop.
% %
% % Reproduces, from autoware_freespace_planning_algorithms:
% %   AstarSearch::search       astar_search.cpp:668
% %   AstarSearch::expandNodes  astar_search.cpp:776
% %   AstarSearch::isGoal       astar_search.cpp:1108
% %   AstarSearch::setPath      astar_search.cpp:1000
% %
% % DEVIATION: the heuristic is plain Euclidean distance. Autoware uses
% %   max(Dijkstra-from-goal, ReedsShepp)  astar_search.cpp:620
% % Ours is still optimistic, just weaker, so the search explores more nodes.
% 
% theta_size = 120;                      % yaml
% curve_w    = 0.5;   
% reverse_w = 0.7;   % curve_weight, reverse_weight
% dir_w      = 2.0;   
% heur_w    = 2.0;   % direction_change_weight, distance_heuristic_weight
% lon_r = 0.25;
% lat_r = 0.15;
% ang_r = deg2rad(2); %goal pose tolerance
% 
% %%Parameters to tune!!
% max_turning_ratio = 0.8;
% turning_steps     = 5;
% step  = 0.5;                           % expansion_distance
% 
% max_iter = 100000;
% 
% 
% 
% res = costmap.res;  ox = costmap.origin(1);  oy = costmap.origin(2);
% W = double(costmap.width);  H = double(costmap.height);
% 
% CAP    = 200000;
% nodes  = zeros(CAP,11);        % x y th g f is_back si dir_dist parent
% closed = false(CAP,1);
% seen   = zeros(W*H*theta_size, 1, 'int32');    % grid key -> node row
% openF  = zeros(CAP,1);  openI = zeros(CAP,1);  on = 0;
% 
% nodes(1,:) = [start_pose(1) start_pose(2) start_pose(3) 0 0 0 0 0 0 0 0];
% nodes(1,5) = heur_w*hypot(start_pose(1)-goal_pose(1), start_pose(2)-goal_pose(2));
% seen(k3(start_pose(1),start_pose(2),start_pose(3))) = 1;
% nn = 1;  on = 1;  openF(1) = nodes(1,5);  openI(1) = 1;
% 
% found = 0;
% for iter = 1:max_iter
%     if on == 0, break; end
%     [~,k] = min(openF(1:on));                       % take the most promising
%     cur = openI(k);
%     openF(k) = openF(on);  openI(k) = openI(on);  on = on - 1;
% 
%     if closed(cur), continue; end
%     closed(cur) = true;
%     if atGoal(cur), found = cur; break; end
% 
%     nxt = nextStates( ...
%     costmap, vehicle, ...
%     nodes(cur,1), nodes(cur,2), nodes(cur,3), ...
%     max_turning_ratio, turning_steps, step);
%     for j = 1:size(nxt,1)
%         nx = nxt(j,1);  
%         ny = nxt(j,2);  
%         nth = nxt(j,3);
%         is_back = nxt(j,4); 
%         front_index = nxt(j,5);
%         rear_index = nxt(j,6);
%         mode = nxt(j,7);
% 
%         kk = k3(nx,ny,nth);  
%         r = seen(kk);
%         if r > 0 && closed(r), continue; end
% 
%         switched = (nodes(cur,11) ~= 0) && (is_back ~= nodes(cur,6));
% 
%         normalized_steering =max(abs(front_index),abs(rear_index))/ turning_steps; %Normalizer in case the turning_steps is greater than 1
%         % front_usage = abs(front_index) / turning_steps;
%         % rear_usage  = abs(rear_index)  / turning_steps;
%         % normalized_steering = 0.5*front_usage + 0.5*rear_usage; %New implementation of the normalizer in order to consider the rear and front contamination
%         w = 1 + curve_w*normalized_steering;                    % getSteeringCost
%         if is_back, w = w*(1 + reverse_w); end      % reverse_weight
%         g = nodes(cur,4) + w*step;
%         if switched, g = g + dir_w*(1 + 1/(1 + nodes(cur,10))); end
%         f = g + heur_w*hypot(nx-goal_pose(1), ny-goal_pose(2));
% 
%         if r == 0 || f < nodes(r,5)
%             if r == 0, nn = nn + 1;  r = nn;  seen(kk) = int32(r); end
%             dd = step;  if ~switched, dd = dd + nodes(cur,10); end
%             nodes(r,:) = [ ...
%                             nx, ny, nth, g, f, double(is_back), ...
%                             front_index, rear_index, mode, dd, cur];
%             on = on + 1;  openF(on) = f;  openI(on) = r;
%         end
%     end
% end
% 
% if found
%     ch = found;
%     while nodes(ch(end),11) ~= 0, ch(end+1) = nodes(ch(end),11); end   %#ok<AGROW>
%     path = nodes(flip(ch), [1 2 3 6]);
%     fprintf('plan found: %.2f m, %d reversals, %d iterations\n', ...
%         sum(hypot(diff(path(:,1)),diff(path(:,2)))), nnz(diff(path(:,4))~=0), iter);
% else
%     path = zeros(0,4);
%     fprintf('no plan after %d iterations\n', iter);
% end
% 
%     function kk = k3(x,y,th)
%         ix = round((x-ox)/res);   iy = round((y-oy)/res);
%         it = mod(round(mod(th,2*pi)/(2*pi/theta_size)), theta_size);
%         kk = it*(W*H) + iy*W + ix + 1;
%     end
% 
%     function tf = atGoal(r)
%         dx =  cos(goal_pose(3))*(nodes(r,1)-goal_pose(1)) + sin(goal_pose(3))*(nodes(r,2)-goal_pose(2));
%         dy = -sin(goal_pose(3))*(nodes(r,1)-goal_pose(1)) + cos(goal_pose(3))*(nodes(r,2)-goal_pose(2));
%         dt = atan2(sin(nodes(r,3)-goal_pose(3)), cos(nodes(r,3)-goal_pose(3)));
%         tf = abs(dx) <= lon_r && abs(dy) <= lat_r && abs(dt) <= ang_r;
%     end
% end



%==========================================================================
% 2WS
% function path = hybridAstar(costmap, vehicle, start_pose, goal_pose)
% %HYBRIDASTAR  The search loop.
% %
% % Reproduces, from autoware_freespace_planning_algorithms:
% %   AstarSearch::search       astar_search.cpp:668
% %   AstarSearch::expandNodes  astar_search.cpp:776
% %   AstarSearch::isGoal       astar_search.cpp:1108
% %   AstarSearch::setPath      astar_search.cpp:1000
% %
% % DEVIATION: the heuristic is plain Euclidean distance. Autoware uses
% %   max(Dijkstra-from-goal, ReedsShepp)  astar_search.cpp:620
% % Ours is still optimistic, just weaker, so the search explores more nodes.
% 
% theta_size = 120;                      % yaml
% curve_w    = 0.5;   reverse_w = 0.7;   % curve_weight, reverse_weight
% dir_w      = 2.0;   heur_w    = 2.0;   % direction_change_weight, distance_heuristic_weight
% 
% lon_r = 0.25;
% lat_r = 0.15;
% ang_r = deg2rad(2); %goal pose tolerances
% 
% step  = 0.5;                           % expansion_distance
% max_iter = 100000;
% 
% res = costmap.res;  ox = costmap.origin(1);  oy = costmap.origin(2);
% W = double(costmap.width);  H = double(costmap.height);
% 
% CAP    = 200000;
% nodes  = zeros(CAP,9);        % x y th g f is_back si dir_dist parent
% closed = false(CAP,1);
% seen   = zeros(W*H*theta_size, 1, 'int32');    % grid key -> node row
% openF  = zeros(CAP,1);  openI = zeros(CAP,1);  on = 0;
% 
% nodes(1,:) = [start_pose(1) start_pose(2) start_pose(3) 0 0 0 0 0 0];
% nodes(1,5) = heur_w*hypot(start_pose(1)-goal_pose(1), start_pose(2)-goal_pose(2));
% seen(k3(start_pose(1),start_pose(2),start_pose(3))) = 1;
% nn = 1;  on = 1;  openF(1) = nodes(1,5);  openI(1) = 1;
% 
% found = 0;
% for iter = 1:max_iter
%     if on == 0, break; end
%     [~,k] = min(openF(1:on));                       % take the most promising
%     cur = openI(k);
%     openF(k) = openF(on);  openI(k) = openI(on);  on = on - 1;
% 
%     if closed(cur), continue; end
%     closed(cur) = true;
%     if atGoal(cur), found = cur; break; end
% 
%     nxt = nextStates(costmap, vehicle, nodes(cur,1), nodes(cur,2), nodes(cur,3));
%     for j = 1:size(nxt,1)
%         nx = nxt(j,1);  ny = nxt(j,2);  nth = nxt(j,3);
%         is_back = nxt(j,4);  si = nxt(j,5);
% 
%         kk = k3(nx,ny,nth);  r = seen(kk);
%         if r > 0 && closed(r), continue; end
% 
%         switched = (nodes(cur,9) ~= 0) && (is_back ~= nodes(cur,6));
% 
%         w = 1 + curve_w*abs(si);                    % getSteeringCost
%         if is_back, w = w*(1 + reverse_w); end      % reverse_weight
%         g = nodes(cur,4) + w*step;
%         if switched, g = g + dir_w*(1 + 1/(1 + nodes(cur,8))); end
%         f = g + heur_w*hypot(nx-goal_pose(1), ny-goal_pose(2));
% 
%         if r == 0 || f < nodes(r,5)
%             if r == 0, nn = nn + 1;  r = nn;  seen(kk) = int32(r); end
%             dd = step;  if ~switched, dd = dd + nodes(cur,8); end
%             nodes(r,:) = [nx ny nth g f is_back si dd cur];
%             on = on + 1;  openF(on) = f;  openI(on) = r;
%         end
%     end
% end
% 
% if found
%     ch = found;
%     while nodes(ch(end),9) ~= 0, ch(end+1) = nodes(ch(end),9); end   %#ok<AGROW>
%     path = nodes(flip(ch), [1 2 3 6]);
%     fprintf('plan found: %.2f m, %d reversals, %d iterations\n', ...
%         sum(hypot(diff(path(:,1)),diff(path(:,2)))), nnz(diff(path(:,4))~=0), iter);
% else
%     path = zeros(0,4);
%     fprintf('no plan after %d iterations\n', iter);
% end
% 
%     function kk = k3(x,y,th)
%         ix = round((x-ox)/res);   iy = round((y-oy)/res);
%         it = mod(round(mod(th,2*pi)/(2*pi/theta_size)), theta_size);
%         kk = it*(W*H) + iy*W + ix + 1;
%     end
% 
%     function tf = atGoal(r)
%         dx =  cos(goal_pose(3))*(nodes(r,1)-goal_pose(1)) + sin(goal_pose(3))*(nodes(r,2)-goal_pose(2));
%         dy = -sin(goal_pose(3))*(nodes(r,1)-goal_pose(1)) + cos(goal_pose(3))*(nodes(r,2)-goal_pose(2));
%         dt = atan2(sin(nodes(r,3)-goal_pose(3)), cos(nodes(r,3)-goal_pose(3)));
%         tf = abs(dx) <= lon_r && abs(dy) <= lat_r && abs(dt) <= ang_r;
%     end
% end