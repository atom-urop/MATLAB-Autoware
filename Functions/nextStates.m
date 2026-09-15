%% 4WS ONLY COUNTER PHASE EXPANSIONS
% function nxt = nextStates( ...
%     costmap, vehicle, x, y, th, ...
%     max_turning_ratio, turning_steps, expansion_distance)
% %NEXTSTATES  The six moves available from one state.
% %
% % Reproduces the move set of AstarSearch::expandNodes  astar_search.cpp:776
% % Parameter values from config/freespace_planner.param.yaml
% %
% % nxt : [n x 4] rows of [x y theta is_back] — collision-free moves only
% 
% % Columns: [front_index, rear_index, mode]
% % mode: 0 = straight, 1 = counter-phase, 2 = in-phase
% 
% L         = vehicle.param.wheel_base;
% 
% max_steer = ...
%     vehicle.param.max_steer_angle * max_turning_ratio;
% 
% steer_res = max_steer / turning_steps;
% 
% % One straight pair plus:
% % 2N counter-phase pairs and 2N in-phase pairs
% steering_pairs = zeros(4*turning_steps + 1, 3);
% 
% row = 1;
% steering_pairs(row,:) = [0, 0, 0];
% 
% for si = -turning_steps:turning_steps
% 
%     if si == 0
%         continue;
%     end
% 
%     % Counter-phase
%     row = row + 1;
%     steering_pairs(row,:) = [
%         si, -si, 1
%     ];
% 
%     % % In-phase
%     % row = row + 1;
%     % steering_pairs(row,:) = [
%     %     si, si, 2
% end
% 
% 
% nxt = zeros(0,7);
% 
% for is_back = [false true]
% 
%     distance = expansion_distance;
%     if is_back
%         distance = -distance;
%     end
% 
%     for k = 1:size(steering_pairs,1)
% 
%         front_index = steering_pairs(k,1);
%         rear_index  = steering_pairs(k,2);
%         mode        = steering_pairs(k,3);
% 
%         steer_f = front_index * steer_res;
%         steer_r = rear_index  * steer_res;
% 
%         [x2,y2,th2] = bicycleGetPose( ...
%             x,y,th,steer_f,steer_r,L,distance);
% 
%         if ~collisionCheck(costmap,vehicle,x2,y2,th2)
%             nxt(end+1,:) = [ ...
%                 x2, y2, th2, double(is_back), ...
%                 front_index, rear_index, mode];
%         end
%     end
% end



%==========================================================================
%% 4WS COUNTER AND INPHASE EXPANSIONS
function nxt = nextStates( ...
    costmap, vehicle, x, y, th, ...
    max_turning_ratio, turning_steps, expansion_distance)
%NEXTSTATES  The six moves available from one state.
%
% Reproduces the move set of AstarSearch::expandNodes  astar_search.cpp:776
% Parameter values from config/freespace_planner.param.yaml
%
% nxt : [n x 4] rows of [x y theta is_back] — collision-free moves only

% Columns: [front_index, rear_index, mode]
% mode: 0 = straight, 1 = counter-phase, 2 = in-phase

L         = vehicle.param.wheel_base;

max_steer = ...
    vehicle.param.max_steer_angle * max_turning_ratio;

steer_res = max_steer / turning_steps;

% One straight pair plus:
% 2N counter-phase pairs and 2N in-phase pairs
steering_pairs = zeros(4*turning_steps + 1, 3);

row = 1;
steering_pairs(row,:) = [0, 0, 0];

for si = -turning_steps:turning_steps

    if si == 0
        continue;
    end

    % Counter-phase
    row = row + 1;
    steering_pairs(row,:) = [
        si, -si, 1
    ];

    % In-phase
    row = row + 1;
    steering_pairs(row,:) = [
        si, si, 2
    ];
end

nxt = zeros(0,7);

for is_back = [false true]

    distance = expansion_distance;
    if is_back
        distance = -distance;
    end

    for k = 1:size(steering_pairs,1)

        front_index = steering_pairs(k,1);
        rear_index  = steering_pairs(k,2);
        mode        = steering_pairs(k,3);

        steer_f = front_index * steer_res;
        steer_r = rear_index  * steer_res;

        [x2,y2,th2] = bicycleGetPose( ...
            x,y,th,steer_f,steer_r,L,distance);

        if ~collisionCheck(costmap,vehicle,x2,y2,th2)
            nxt(end+1,:) = [ ...
                x2, y2, th2, double(is_back), ...
                front_index, rear_index, mode];
        end
    end
end


%==========================================================================
% %% 2WS
% function nxt = nextStates(costmap, vehicle, x, y, th)
% %NEXTSTATES  The six moves available from one state.
% %
% % Reproduces the move set of AstarSearch::expandNodes  astar_search.cpp:776
% % Parameter values from config/freespace_planner.param.yaml
% %
% % nxt : [n x 4] rows of [x y theta is_back] — collision-free moves only
% 
% max_turning_ratio  = 0.5;      % yaml
% turning_steps      = 1;        % yaml
% expansion_distance = 0.5;      % yaml  [m]
% 
% L         = vehicle.param.wheel_base;
% max_steer = vehicle.param.max_steer_angle * max_turning_ratio;
% steer_res = max_steer / turning_steps;
% 
% nxt = zeros(0,5);
% for is_back = [false true]
%     d = expansion_distance;
%     if is_back, d = -d; end
%     for si = -turning_steps : turning_steps
%         [x2, y2, th2] = bicycleGetPose(x, y, th, si*steer_res, L, d);
%         if ~collisionCheck(costmap, vehicle, x2, y2, th2)
%             nxt(end+1,:) = [x2 y2 th2 double(is_back) si];    %#ok<AGROW>
%         end
%     end
% end
% end