%%Unified 4W for all cases
function nxt = nextStates( ...
    costmap, vehicle, x, y, th, ...
    max_turning_ratio, turning_steps, expansion_distance, ...
    steering_configuration)%unified 4WS 3 cases in one function
%NEXTSTATES Generate collision-free Hybrid A* successor states.
%
% steering_configuration:
%   1 = counter-phase only
%   2 = counter-phase and in-phase
%   3 = every front/rear combination (4WIS)
%
% Output columns:
% [x, y, theta, is_back, front_index, rear_index, mode]
%
% mode:
%   0 = straight
%   1 = counter-phase
%   2 = in-phase
%   3 = rear steering only
%   4 = front steering only

L = vehicle.param.wheel_base;

max_steer = ...
    vehicle.param.max_steer_angle * max_turning_ratio;

steer_res = max_steer / turning_steps;

%% Construct the selected steering set

switch steering_configuration

    case 1
        % Straight plus 2N counter-phase commands.
        number_of_pairs = 2*turning_steps + 1;
        steering_pairs = zeros(number_of_pairs,3);

        row = 1;
        steering_pairs(row,:) = [0,0,0];

        for steering_index = -turning_steps:turning_steps

            if steering_index == 0
                continue;
            end

            row = row + 1;

            steering_pairs(row,:) = [ ...
                steering_index, ...
                -steering_index, ...
                1];
        end

    case 2
        % Straight plus counter-phase and in-phase commands.
        number_of_pairs = 4*turning_steps + 1;
        steering_pairs = zeros(number_of_pairs,3);

        row = 1;
        steering_pairs(row,:) = [0,0,0];

        for steering_index = -turning_steps:turning_steps

            if steering_index == 0
                continue;
            end

            % Counter-phase.
            row = row + 1;

            steering_pairs(row,:) = [ ...
                steering_index, ...
                -steering_index, ...
                1];

            % In-phase.
            row = row + 1;

            steering_pairs(row,:) = [ ...
                steering_index, ...
                steering_index, ...
                2];
        end

    case 3
        % Every combination of front and rear steering.
        number_of_pairs = (2*turning_steps + 1)^2;
        steering_pairs = zeros(number_of_pairs,3);

        % Put straight first for consistent ordering.
        row = 1;
        steering_pairs(row,:) = [0,0,0];

        for front_index = -turning_steps:turning_steps
            for rear_index = -turning_steps:turning_steps

                if front_index == 0 && rear_index == 0
                    continue;

                elseif front_index == 0
                    mode = 3;

                elseif rear_index == 0
                    mode = 4;

                elseif sign(front_index) ~= sign(rear_index)
                    mode = 1;

                else
                    mode = 2;
                end

                row = row + 1;

                steering_pairs(row,:) = [ ...
                    front_index, ...
                    rear_index, ...
                    mode];
            end
        end

    otherwise
        error( ...
            'nextStates:InvalidSteeringConfiguration', ...
            'steering_configuration must be 1, 2, or 3.');
end

%% Common motion and collision-checking implementation

maximum_number_of_successors = ...
    2*size(steering_pairs,1);

nxt = zeros(maximum_number_of_successors,7);
number_of_successors = 0;

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
            x,y,th, ...
            steer_f,steer_r,L,distance);

        % Check the complete movement at intervals no larger than
        % half a costmap cell.
        number_of_collision_samples = max( ...
            2, ...
            ceil(abs(distance)/(costmap.res/2)));

        edge_is_collision_free = true;

        for collision_sample_index = ...
                1:number_of_collision_samples

            sample_distance = ...
                distance * ...
                collision_sample_index / ...
                number_of_collision_samples;

            [sample_x,sample_y,sample_theta] = ...
                bicycleGetPose( ...
                    x,y,th, ...
                    steer_f,steer_r,L, ...
                    sample_distance);

            if collisionCheck( ...
                    costmap,vehicle, ...
                    sample_x,sample_y,sample_theta)

                edge_is_collision_free = false;
                break;
            end
        end

        if edge_is_collision_free

            number_of_successors = ...
                number_of_successors + 1;

            nxt(number_of_successors,:) = [ ...
                x2, y2, th2, ...
                double(is_back), ...
                front_index, ...
                rear_index, ...
                mode];
        end
    end
end

nxt = nxt(1:number_of_successors,:);
end


% %% ~4IWS
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
% % One straight pair plus, for every nonzero steering index:
% % - 2N counter-phase pairs
% % - 2N in-phase pairs
% % - 2N rear-only pairs
% % mode:
% % 0 = straight
% % 1 = counter-phase
% % 2 = same-direction/in-phase
% % 3 = rear steering only
% % 4 = front steering only
% 
% % Every possible combination of front and rear steering indices
% 
% number_of_pairs = (2*turning_steps + 1)^2;
% steering_pairs = zeros(number_of_pairs, 3);
% 
% % Put straight first
% row = 1;
% steering_pairs(row,:) = [0, 0, 0];
% 
% for front_index = -turning_steps:turning_steps
% 
%     for rear_index = -turning_steps:turning_steps
% 
%         if front_index == 0 && rear_index == 0
%             continue;
%         end
% 
%         if front_index == 0
%             mode = 3;  % rear-only
% 
%         elseif rear_index == 0
%             mode = 4;  % front-only
% 
%         elseif sign(front_index) ~= sign(rear_index)
%             mode = 1;  % counter-phase
% 
%         else
%             mode = 2;  % same-direction/in-phase
%         end
% 
%         row = row + 1;
%         steering_pairs(row,:) = [ ...
%             front_index, rear_index, mode];
%     end
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
%         % [x2,y2,th2] = bicycleGetPose( ...
%         %     x,y,th,steer_f,steer_r,L,distance);
%         % 
%         % if ~collisionCheck(costmap,vehicle,x2,y2,th2)
%         %     nxt(end+1,:) = [ ...
%         %         x2, y2, th2, double(is_back), ...
%         %         front_index, rear_index, mode];
%         % end
%         % Calculate the final pose of this expansion.
%         [x2,y2,th2] = bicycleGetPose( ...
%             x,y,th,steer_f,steer_r,L,distance);
% 
%         % Check the movement at intervals no larger than half a costmap cell.
%         number_of_collision_samples = max( ...
%             2, ...
%             ceil(abs(distance)/(costmap.res/2)));
% 
%         edge_is_collision_free = true;
% 
%         for collision_sample_index = 1:number_of_collision_samples
% 
%             sample_distance = ...
%                 distance * ...
%                 collision_sample_index / ...
%                 number_of_collision_samples;
% 
%             [sample_x,sample_y,sample_theta] = bicycleGetPose( ...
%                 x,y,th, ...
%                 steer_f,steer_r,L, ...
%                 sample_distance);
% 
%             if collisionCheck( ...
%                     costmap,vehicle, ...
%                     sample_x,sample_y,sample_theta)
% 
%                 edge_is_collision_free = false;
%                 break;
%             end
%         end
% 
%         % Add the successor only if the complete movement is collision-free.
%         if edge_is_collision_free
% 
%             nxt(end+1,:) = [ ...
%                 x2, y2, th2, double(is_back), ...
%                 front_index, rear_index, mode];
%         end
%     end
% end




%==========================================================================
%% 4WS COUNTER AND INPHASE EXPANSIONS
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
%     % In-phase
%     row = row + 1;
%     steering_pairs(row,:) = [
%         si, si, 2
%     ];
% end
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
% 4WS ONLY COUNTER PHASE EXPANSIONS
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
% % Straight plus 2*turning_steps counter-phase commands
% steering_pairs = zeros(2*turning_steps + 1, 3);
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
% %% 2WS
% function nxt = nextStates(costmap, vehicle, x, y, th)
% %NEXTSTATES  The six moves available from one state.
% %
% % Reproduces the move set of AstarSearch::expandNodes  astar_search.cpp:776
% % Parameter values from config/freespace_planner.param.yaml
% %
% % nxt : [n x 4] rows of [x y theta is_back] — collision-free moves only
% 
% max_turning_ratio  = 0.8;      % yaml
% turning_steps      = 5;        % yaml
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
%         [x2, y2, th2] = bicycleGetPose( ...
%     x, y, th, si*steer_res, 0, L, d);
%         if ~collisionCheck(costmap, vehicle, x2, y2, th2)
%             nxt(end+1,:) = [x2 y2 th2 double(is_back) si];    %#ok<AGROW>
%         end
%     end
% end
% end
