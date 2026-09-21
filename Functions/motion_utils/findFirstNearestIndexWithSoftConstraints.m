function nearest_idx = findFirstNearestIndexWithSoftConstraints( ...
    trajectory,...
    ego_pose,...
    dist_threshold,...
    yaw_threshold)


%% ============================================================
% FIND FIRST NEAREST INDEX WITH SOFT CONSTRAINTS
%
% Equivalent to:
%
% autoware::motion_utils::
% findFirstNearestIndexWithSoftConstraints()
%
% Search strategy:
%
% 1) distance + yaw constraint
% 2) distance constraint only
% 3) yaw constraint only
% 4) pure nearest point
%
% ============================================================


N = length(trajectory.pose.position.x);



%% Extract trajectory yaw

traj_yaw = QuaternionToYaw( ...
    trajectory.pose.orientation.qx,...
    trajectory.pose.orientation.qy,...
    trajectory.pose.orientation.qz,...
    trajectory.pose.orientation.qw);



%% Ego yaw

ego_yaw = QuaternionToYaw( ...
    ego_pose.pose.orientation.x,...
    ego_pose.pose.orientation.y,...
    ego_pose.pose.orientation.z,...
    ego_pose.pose.orientation.w);



%% Ego position

ego_x = ego_pose.pose.position.x;
ego_y = ego_pose.pose.position.y;



%% ============================================================
% CASE 1:
% distance + yaw constraints
% =============================================================


idx = searchNearest( ...
    trajectory,...
    traj_yaw,...
    ego_x,...
    ego_y,...
    ego_yaw,...
    dist_threshold,...
    yaw_threshold,...
    true,...
    true);


if idx >= 0

    nearest_idx = idx;
    return

end



%% ============================================================
% CASE 2:
% distance only
% =============================================================


idx = searchNearest( ...
    trajectory,...
    traj_yaw,...
    ego_x,...
    ego_y,...
    ego_yaw,...
    dist_threshold,...
    yaw_threshold,...
    true,...
    false);


if idx >=0

    nearest_idx=idx;
    return

end



%% ============================================================
% CASE 3:
% yaw only
% =============================================================


idx = searchNearest( ...
    trajectory,...
    traj_yaw,...
    ego_x,...
    ego_y,...
    ego_yaw,...
    dist_threshold,...
    yaw_threshold,...
    false,...
    true);


if idx>=0

    nearest_idx=idx;
    return

end



%% ============================================================
% CASE 4:
% no constraint
% =============================================================


min_dist=inf;
nearest_idx=1;


for i=1:N

    dx=trajectory.pose.position.x(i)-ego_x;
    dy=trajectory.pose.position.y(i)-ego_y;

    d=dx^2+dy^2;


    if d < min_dist

        min_dist=d;
        nearest_idx=i;

    end

end


end