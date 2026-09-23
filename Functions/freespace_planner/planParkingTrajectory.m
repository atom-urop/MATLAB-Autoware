% function T = planParkingTrajectory(msg, start_pose, goal_pose, veh)
% %PLANPARKINGTRAJECTORY  Occupancy grid in -> 100-point trajectory out.
% %
% % Called by the freespace_planner Simulink block. Runs once per simulation,
% % exactly like the Lane Follower's trajectoryGenerator.
% %
% % Reproduces the freespace_planner node's job end to end:
% %   AstarSearch::makePlan / search        astar_search.cpp
% %   partial-trajectory split              autoware_freespace_planner/utils.cpp
% %
% %   msg        : nav_msgs/OccupancyGrid payload from the costmap block
% %   start_pose : [x y theta]
% %   goal_pose  : [x y theta]
% %   veh        : vehicle parameter vector (the same one the plot block uses)
% %   T          : [100 x 13] = x y z qx qy qz qw vx vy ax wz df dr
% 
% N = 100;
% T = zeros(N,13);
% 
% % --- rebuild the structs the planner functions expect -------------------
% cm.res    = double(msg.info.resolution);
% cm.width  = double(msg.info.width);
% cm.height = double(msg.info.height);
% cm.origin = [double(msg.info.origin.position.x), ...
%     double(msg.info.origin.position.y)];
% cm.grid   = double(msg.data)/100;        % bus carries 0/100, planner wants 0/1
% drawOccupancyGrid(msg);                  % show the grid the planner received
% 
% vehicle.param.wheel_base      = veh(3);
% vehicle.param.wheel_tread     = veh(4);
% vehicle.param.front_overhang  = veh(5);
% vehicle.param.rear_overhang   = veh(6);
% vehicle.param.left_overhang   = veh(7);
% vehicle.param.right_overhang  = veh(8);
% vehicle.param.max_steer_angle = veh(10);
% 
% % Make the current map available even if planning fails
% assignin('base', 'costmap_active', struct('msg', msg));
% 
% % Remove any path left by a previous simulation
% % assignin('base', 'parking_path', zeros(0,4));
% assignin('base', 'parking_path', zeros(0,7));
% 
% % --- plan ---------------------------------------------------------------
% path = hybridAstar(cm, vehicle, start_pose(:)', goal_pose(:)');
% 
% 
% if isempty(path)
%     warning('planParkingTrajectory: no plan found - emitting a zero trajectory');
%     return;
% end
% 
% % keep the plan visible for plotting / inspection after the run
% assignin('base', 'parking_path', path);
% assignin('base', 'costmap_active', struct('msg', msg));
% 
% % --- first gear segment -> the 13 bus signals ---------------------------
% [x,y,z,qx,qy,qz,qw,vx,vy,ax,wz,df,dr] = pathToTrajectory(path, 1);
% T = [x y z qx qy qz qw vx vy ax wz df dr];
% assignin('base','plan_traj', T);
% end

function [T, segment_count] = planParkingTrajectory( ...
    msg, start_pose, goal_pose, veh, segment_request, reset_plan)
%PLANPARKINGTRAJECTORY
% Compute the complete parking path once and return one requested
% forward/reverse segment as a 100-point trajectory.

N = 100;

T = zeros(N,13);
segment_count = 0;

persistent cached_path

% Preserve compatibility with calls that do not provide the new inputs.
if nargin < 5 || isempty(segment_request)
    segment_request = 1;
end

if nargin < 6
    reset_plan = true;
end

% Run Hybrid A* only for a new planning request.
if reset_plan || isempty(cached_path)

    % Rebuild the costmap structure expected by the planner.
    cm.res    = double(msg.info.resolution);
    cm.width  = double(msg.info.width);
    cm.height = double(msg.info.height);
    cm.origin = [ ...
        double(msg.info.origin.position.x), ...
        double(msg.info.origin.position.y)];

    cm.grid = double(msg.data)/100;

    drawOccupancyGrid(msg);

    % Rebuild the vehicle structure expected by Hybrid A*.
    vehicle.param.wheel_base      = veh(3);
    vehicle.param.wheel_tread     = veh(4);
    vehicle.param.front_overhang  = veh(5);
    vehicle.param.rear_overhang   = veh(6);
    vehicle.param.left_overhang   = veh(7);
    vehicle.param.right_overhang  = veh(8);
    vehicle.param.max_steer_angle = veh(10);

    assignin('base', 'costmap_active', struct('msg', msg));
    assignin('base', 'parking_path', zeros(0,7));

    % This is the slow operation. It runs only for a new plan.
    cached_path = hybridAstar( ...
        cm, vehicle, start_pose(:)', goal_pose(:)');

    if isempty(cached_path)
        warning(['planParkingTrajectory: no plan found - ', ...
                 'emitting a zero trajectory']);
        return;
    end

    assignin('base', 'parking_path', cached_path);
    assignin('base', 'costmap_active', struct('msg', msg));
end

% Use the already-computed complete path.
path = cached_path;

if isempty(path)
    return;
end

% Apply the same first-point convention used by pathToTrajectory.
% The initial point inherits the direction and steering of the first move.
path_for_count = path;

if size(path_for_count,1) >= 2
    number_of_columns = min(6, size(path_for_count,2));

    path_for_count(1,4:number_of_columns) = ...
        path_for_count(2,4:number_of_columns);
end

% Count the forward/reverse segments in the complete path.
direction_changes = ...
    find(diff(path_for_count(:,4)) ~= 0);

segment_count = double(numel(direction_changes) + 1);

% Convert the request into a valid integer segment number.
segment_index = round(double(segment_request));

segment_index = max(1, segment_index);
segment_index = min(segment_index, segment_count);

% Convert only the requested segment to the 100-point trajectory bus.
[x,y,z,qx,qy,qz,qw,vx,vy,ax,wz,df,dr] = ...
    pathToTrajectory(path, segment_index);

T = [x y z qx qy qz qw vx vy ax wz df dr];

% Keep these values available for inspection after simulation.
assignin('base', 'plan_traj', T);
assignin('base', 'parking_segment_count', segment_count);
assignin('base', 'parking_active_segment', segment_index);
end