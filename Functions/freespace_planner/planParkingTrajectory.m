function T = planParkingTrajectory(msg, start_pose, goal_pose, veh)
%PLANPARKINGTRAJECTORY  Occupancy grid in -> 100-point trajectory out.
%
% Called by the freespace_planner Simulink block. Runs once per simulation,
% exactly like the Lane Follower's trajectoryGenerator.
%
% Reproduces the freespace_planner node's job end to end:
%   AstarSearch::makePlan / search        astar_search.cpp
%   partial-trajectory split              autoware_freespace_planner/utils.cpp
%
%   msg        : nav_msgs/OccupancyGrid payload from the costmap block
%   start_pose : [x y theta]
%   goal_pose  : [x y theta]
%   veh        : vehicle parameter vector (the same one the plot block uses)
%   T          : [100 x 13] = x y z qx qy qz qw vx vy ax wz df dr

N = 100;
T = zeros(N,13);

% --- rebuild the structs the planner functions expect -------------------
cm.res    = double(msg.info.resolution);
cm.width  = double(msg.info.width);
cm.height = double(msg.info.height);
cm.origin = [double(msg.info.origin.position.x), ...
    double(msg.info.origin.position.y)];
cm.grid   = double(msg.data)/100;        % bus carries 0/100, planner wants 0/1
drawOccupancyGrid(msg);                  % show the grid the planner received

vehicle.param.wheel_base      = veh(3);
vehicle.param.wheel_tread     = veh(4);
vehicle.param.front_overhang  = veh(5);
vehicle.param.rear_overhang   = veh(6);
vehicle.param.left_overhang   = veh(7);
vehicle.param.right_overhang  = veh(8);
vehicle.param.max_steer_angle = veh(10);

% Make the current map available even if planning fails
assignin('base', 'costmap_active', struct('msg', msg));

% Remove any path left by a previous simulation
% assignin('base', 'parking_path', zeros(0,4));
assignin('base', 'parking_path', zeros(0,7));

% --- plan ---------------------------------------------------------------
path = hybridAstar(cm, vehicle, start_pose(:)', goal_pose(:)');


if isempty(path)
    warning('planParkingTrajectory: no plan found - emitting a zero trajectory');
    return;
end

% keep the plan visible for plotting / inspection after the run
assignin('base', 'parking_path', path);
assignin('base', 'costmap_active', struct('msg', msg));

% --- first gear segment -> the 13 bus signals ---------------------------
[x,y,z,qx,qy,qz,qw,vx,vy,ax,wz,df,dr] = pathToTrajectory(path, 1);
T = [x y z qx qy qz qw vx vy ax wz df dr];
assignin('base','plan_traj', T);
end