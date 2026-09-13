function [x2, y2, th2] = bicycleGetPose(x, y, th, steer, base_length, distance)
%BICYCLEGETPOSE  Where does the car end up after one move?
%
% Literal transcription of, in autoware_freespace_planning_algorithms:
%   kinematic_bicycle_model::getPose           kinematic_bicycle_model.hpp
%   kinematic_bicycle_model::getTurningRadius  kinematic_bicycle_model.hpp
%
% distance may be negative -> reverse.

%% Currently we are still considering the autoware;s bicycle model

if abs(steer) < 0.001                      % straight
    x2  = x + distance*cos(th);
    y2  = y + distance*sin(th);
    th2 = th;
    return;
end

R    = base_length / tan(steer);           % turning radius
beta = distance / R;                       % heading change over this arc
x2   = x + (R*sin(th + beta) - R*sin(th));
y2   = y + (R*cos(th)        - R*cos(th + beta));
th2  = th + beta;
end