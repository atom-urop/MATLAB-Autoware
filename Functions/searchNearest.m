function idx = searchNearest( ...
    trajectory,...
    traj_yaw,...
    ego_x,...
    ego_y,...
    ego_yaw,...
    dist_threshold,...
    yaw_threshold,...
    use_dist,...
    use_yaw)


% ======================================
%
% We replicate the same logics in Autoware for the three block strategies
% in findFirstNearestIndexWithConstraints.
%
% ======================================

N=length(trajectory.pose.position.x);


min_squared_dist=inf;

idx=-1;


for i=1:N


    dx=trajectory.pose.position.x(i)-ego_x;
    dy=trajectory.pose.position.y(i)-ego_y;


    squared_dist=dx^2+dy^2;



    if use_dist

        if squared_dist > dist_threshold^2
            continue
        end

    end



    if use_yaw

        % CALCYAWDEVIATION
        % Equivalent to Autoware:
        % autoware_utils_geometry::calc_yaw_deviation()

        yaw_error = traj_yaw(i)-ego_yaw;

        % normalize to [-pi, pi]
        yaw_error = mod(yaw_error + pi, 2*pi) - pi;


        if abs(yaw_error)>yaw_threshold

            continue

        end

    end



    if squared_dist < min_squared_dist

        min_squared_dist=squared_dist;

        idx=i;

    end


end


end