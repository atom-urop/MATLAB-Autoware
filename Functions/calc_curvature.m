function curvature = calc_curvature(p1, p2, p3)

% calc_curvature
%
% Computes the geometric curvature of three points using the
% Menger curvature formulation.
%
% This function reproduces:
%
% autoware_motion_utils::calc_curvature()
%
% Original Autoware implementation:
%
% double calc_curvature(
%   const geometry_msgs::msg::Point & p1,
%   const geometry_msgs::msg::Point & p2,
%   const geometry_msgs::msg::Point & p3)
%
% The curvature is computed from three points:
%
%       p1 -------- p2 -------- p3
%
% and corresponds to the inverse radius of the circumcircle passing
% through the three points.
%
% Formula:
%
%   k = 2 * ((x2-x1)(y3-y1) - (y2-y1)(x3-x1))
%       -------------------------------------------------
%       |p1-p2| |p2-p3| |p3-p1|
%
% where:
%
%   |pi-pj| is the 2D Euclidean distance between two points.
%
% Reference:
%
% Menger curvature:
% https://en.wikipedia.org/wiki/Menger_curvature
%
%
% Inputs
% -------
% p1 : [x y z]
%     First geometry_msgs/msg/Point.
%
% p2 : [x y z]
%     Middle geometry_msgs/msg/Point.
%
% p3 : [x y z]
%     Third geometry_msgs/msg/Point.
%
%
% Outputs
% --------
% curvature : scalar
%     Signed curvature [1/m].
%
%
% Difference from Autoware:
%
% Autoware checks:
%
%   if (fabs(denominator) < 1e-10)
%       throw runtime_error(...)
%
% MATLAB/Simulink code generation does not support exceptions.
% Therefore, degenerate configurations return curvature = 0.

%% Compute 2D distances

d12 = sqrt( ...
    (p2(1)-p1(1))^2 + ...
    (p2(2)-p1(2))^2 );


d23 = sqrt( ...
    (p3(1)-p2(1))^2 + ...
    (p3(2)-p2(2))^2 );


d31 = sqrt( ...
    (p1(1)-p3(1))^2 + ...
    (p1(2)-p3(2))^2 );


%% Denominator

denominator = d12 * d23 * d31;


%% Degenerate geometry handling

% Autoware:
%
% if (fabs(denominator) < 1e-10)
% {
%     throw runtime_error(
%       "points are too close for curvature calculation.");
% }
%
% Simulink implementation:
%
% Return zero curvature.

if abs(denominator) < 1e-10

    curvature = 0.0;
    return;

end


%% Menger curvature

curvature = 2.0 * ( ...
    (p2(1)-p1(1)) * (p3(2)-p1(2)) - ...
    (p2(2)-p1(2)) * (p3(1)-p1(1)) ...
    ) / denominator;


end