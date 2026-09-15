%% 4WS in all cases
function [x2, y2, th2] = bicycleGetPose(x, y, th, steer_f, steer_r, base_length, distance)
%BICYCLEGETPOSE  Where does the car end up after one move?
%
% Literal transcription of, in autoware_freespace_planning_algorithms:
%   kinematic_bicycle_model::getPose           kinematic_bicycle_model.hpp
%   kinematic_bicycle_model::getTurningRadius  kinematic_bicycle_model.hpp
%
% distance may be negative -> reverse.

%% Currently we are still considering the autoware;s bicycle model

if abs(steer_f) < 0.001 && abs(steer_r) < 0.001       %Modified for 4WS             % straight
    x2  = x + distance*cos(th);
    y2  = y + distance*sin(th);
    th2 = th;
    return;
end

Lr = base_length / 2;
Lf = base_length / 2;
L = base_length;
beta = atan((Lr*tan(steer_f) + Lf*tan(steer_r)) / (Lf + Lr));  %Modified for 4WS          % heading change over this arc
%%4WS curvature
kappa = (tan(steer_f) - tan(steer_r)) * cos(beta) / L;

if abs(kappa) < 1e-10
    x2 = x + distance*cos(th + beta);
    y2 = y + distance*sin(th + beta);
    th2 = th;
    return;
end

%%Turning radius
%R = 1 / abs(kappa);

dtheta = distance * kappa;

th2 = th + dtheta;

%%Exact circular-arc position update
x2 = x + (sin(th2 + beta) - sin(th + beta)) / kappa;

y2 = y - (cos(th2 + beta) - cos(th + beta)) / kappa;

end



%==========================================================================
%% 2WS
% function [x2, y2, th2] = bicycleGetPose(x, y, th, steer, base_length, distance)
% %BICYCLEGETPOSE  Where does the car end up after one move?
% %
% % Literal transcription of, in autoware_freespace_planning_algorithms:
% %   kinematic_bicycle_model::getPose           kinematic_bicycle_model.hpp
% %   kinematic_bicycle_model::getTurningRadius  kinematic_bicycle_model.hpp
% %
% % distance may be negative -> reverse.
% 
% %% Currently we are still considering the autoware;s bicycle model
% 
% if abs(steer) < 0.001                      % straight
%     x2  = x + distance*cos(th);
%     y2  = y + distance*sin(th);
%     th2 = th;
%     return;
% end
% 
% R    = base_length / tan(steer);           % turning radius
% beta = distance / R;                       % heading change over this arc
% x2   = x + (R*sin(th + beta) - R*sin(th));
% y2   = y + (R*cos(th)        - R*cos(th + beta));
% th2  = th + beta;
% end