function velocity_limit = ComputeVelocityLimitFromSteerRate( ...
    local_ratio, ...
    ratio_limits, ...
    velocity_thresholds, ...
    steering_angle_rate_limits)


% ComputeVelocityLimitFromSteerRate
%
% Equivalent to Autoware:
%
% SmootherBase::computeVelocityLimitFromSteerRate()
%
% This function computes the maximum allowed velocity according to
% the steering rate constraint.
%
% Autoware input:
%
%   local_steer_rate_velocity_ratio
%
% corresponds to:
%
%   steering angle variation / travelled distance
%
% [rad/m]
%
%
% The velocity limit is obtained from:
%
%       v_limit = steering_rate_limit / ratio
%
%
% where:
%
%       steering_rate_limit [rad/s]
%
%       ratio               [rad/m]
%
%
% The function uses the same lookup logic as Autoware:
%
% computeVelocityLimit()
%
%


%% Equivalent of C++ lambda:
%
% auto compute_velocity = [](double rate_deg, double ratio) {
%     return deg2rad(rate_deg) / ratio;
% };


compute_velocity = @(rate_deg,ratio) ...
    deg2rad(rate_deg) ./ ratio;



%% Iterate through ratio limits in reverse order
%
% Equivalent:
%
% for (int i = ratio_limits.size()-1; i >=0; --i)
%


N = size(ratio_limits,1);


for i = N:-1:1


    % C++:
    %
    % const double lower = ratio_limits[i].second;
    % const double higher = ratio_limits[i].first;


    lower  = ratio_limits(i,2);
    higher = ratio_limits(i,1);



    %
    % If ratio is higher than this range
    %
    % C++:
    %
    % if(local_ratio > higher)
    %     continue;
    %


    if local_ratio > higher

        continue

    end



    %
    % If ratio is below lower bound
    %
    %


    if local_ratio <= lower


        %
        % Last interval:
        %
        % return std::numeric_limits<double>::max()
        %
        % Equivalent MATLAB:
        %
        % realmax
        %

        if i == N

            velocity_limit = realmax;

        else

            %
            % return velocity_thresholds[i]
            %

            velocity_limit = velocity_thresholds(i);

        end


        return

    end



    %
    % Ratio is inside current interval
    %
    %


    if i == N

        current_threshold = ...
            steering_angle_rate_limits(end);

    else

        current_threshold = ...
            steering_angle_rate_limits(i);

    end



    velocity_limit = ...
        compute_velocity( ...
            current_threshold,...
            local_ratio);


    return

end



%
% Equivalent:
%
% return 0.0;
%


velocity_limit = 0.0;


end