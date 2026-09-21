function [steer_rate_velocity_ratio_arr, output] = ComputeSteeringRateAngle4WS(trajectory, ...
    curvature, veh, points_interval, k_ref_LUT, rr_LUT, delta_f_LUT)
% This function is the same function that is inside step2 in the velocity
% smoother in the simulink model, I have added it here in the functions for
% mechanical reasons, in order to be able to produce the values for
% comparison with the Cpp
N = length(curvature);
rr = zeros(N,1);
front_angle = zeros(N,1);
rear_angle  = zeros(N,1);

for i = 1:N
    if curvature(i) > max(k_ref_LUT)
        rr(i) = -1;
        front_angle(i) = 0.7;
        rear_angle(i) = -0.7;
    else
        rr(i) = interp1(k_ref_LUT, rr_LUT, curvature(i));
        front_angle(i) = interp1(k_ref_LUT, delta_f_LUT, curvature(i));
        rear_angle(i) = rr(i) * front_angle(i);
    end
end

trajectory.front_wheel_angle_rad = front_angle;
trajectory.rear_wheel_angle_rad  = rear_angle;

steer_rate_velocity_ratio_arr = zeros(N,1);
for i = 1:N-1
    front_diff = abs(front_angle(i+1)-front_angle(i));
    rear_diff  = abs(rear_angle(i+1)-rear_angle(i));
    steering_diff = max(front_diff, rear_diff);
    eps = 2.220446049250313e-16;
    steer_rate_velocity_ratio_arr(i) = steering_diff/(points_interval+eps);
end
steer_rate_velocity_ratio_arr(end) = steer_rate_velocity_ratio_arr(end-1);

output = trajectory;
end