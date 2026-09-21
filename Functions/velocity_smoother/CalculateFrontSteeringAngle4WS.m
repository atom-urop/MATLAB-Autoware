function front_angle  = CalculateFrontSteeringAngle4WS( ...
    curvature, ...
    wheel_base, ...
    rear_steering_ratio)


% ComputeSteeringAngle4WS
%
% 
%
% Computes front steering angles for a four-wheel-steering vehicle.
%
% The curvature model is:
%
%       kappa = (tan(delta_f)-tan(delta_r))/L
%
% where:
%
%       delta_r = rear_steering_ratio * delta_f
%
%
% Inputs
% -------
%
% curvature:
%       trajectory curvature [1/m]
%
% wheel_base:
%       vehicle wheel base [m]
%
% rear_steering_ratio:
%       ratio between rear and front steering angle.
%
%
% Outputs
% --------
%
% front_angle:
%       front wheel steering angle [rad]



% ATTENTION!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
% This full function doesn't currenty exist in original Autoware.
% It's created for 4WS implementation. The same format (separated function recalled in ApplySteeringRateLimit) can be 
% reported in Autoware when implementing 4WS.


ratio = rear_steering_ratio;


% Steering limits
low  = -0.6;
high =  0.6;


% Binary search

for k = 1:50

    mid = (low+high)/2;


    rear = ratio * mid;


    curvature_est = ...
        (tan(mid)-tan(rear))/wheel_base;


    if curvature_est < curvature

        low = mid;

    else

        high = mid;

    end

end


front_angle = (low+high)/2;


end
