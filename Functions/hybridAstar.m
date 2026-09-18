%==========================================================================
%% 4WS with new approach (new cost function) : added a term for the steering rate + added costs around obstacles
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
heur_w    = 2.0;   % direction_change_weight, distance_heuristic_weight
steer_change_w = 0.1; % The weight used for the steering rate 
clearance_w = 0.5;          % Experimental starting weight to an obstacle
clearance_preferred = 0.6;  % Preferred additional clearance to an obstacle [m]


lon_r = 0.25;
lat_r = 0.15;
ang_r = deg2rad(2); %goal pose tolerance

%%Parameters to tune!!
max_turning_ratio = 0.8;
turning_steps     = 5;
step  = 0.5;                           % expansion_distance

max_iter = 100000;



res = costmap.res;  ox = costmap.origin(1);  oy = costmap.origin(2);
W = double(costmap.width);  H = double(costmap.height);

CAP    = 200000;
nodes  = zeros(CAP,11);        % x y th g f is_back si dir_dist parent
closed = false(CAP,1);
seen   = zeros(W*H*theta_size, 1, 'int32');    % grid key -> node row
openF  = zeros(CAP,1);  openI = zeros(CAP,1);  on = 0;

nodes(1,:) = [start_pose(1) start_pose(2) start_pose(3) 0 0 0 0 0 0 0 0];
nodes(1,5) = heur_w*hypot(start_pose(1)-goal_pose(1), start_pose(2)-goal_pose(2));
seen(k3(start_pose(1),start_pose(2),start_pose(3))) = 1;
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



        f = g + heur_w*hypot(nx-goal_pose(1), ny-goal_pose(2)); %%The complete cost computation when adding the cummulative g cost with the heuristic cost that is f = g + 2*h


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
    path = nodes(flip(ch), [1 2 3 6]);
    fprintf('plan found: %.2f m, %d reversals, %d iterations\n', ...
        sum(hypot(diff(path(:,1)),diff(path(:,2)))), nnz(diff(path(:,4))~=0), iter);
else
    path = zeros(0,4);
    fprintf('no plan after %d iterations\n', iter);
end

    function kk = k3(x,y,th)
        ix = round((x-ox)/res);   iy = round((y-oy)/res);
        it = mod(round(mod(th,2*pi)/(2*pi/theta_size)), theta_size);
        kk = it*(W*H) + iy*W + ix + 1;
    end

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