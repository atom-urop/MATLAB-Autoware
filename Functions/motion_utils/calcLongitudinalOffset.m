function offset = calcLongitudinalOffset( ...
    p_front,...
    p_back,...
    p_target)


%% ============================================================
% CALC LONGITUDINAL OFFSET
%
% Equivalent to Autoware:
%
% motion_utils::calcLongitudinalOffset()
%
% Computes the signed longitudinal distance of a target point
% projected on the trajectory segment.
%
%
% Inputs:
%
% p_front :
%   first point of trajectory segment [x;y]
%
% p_back :
%   second point of trajectory segment [x;y]
%
% p_target :
%   target point (front axle position) [x;y]
%
%
% Output:
%
% offset:
%   signed distance along the segment
%
%
% Positive:
%   target projection is after p_front
%
% Negative:
%   target projection is before p_front
%
% ============================================================



%% Segment vector

segment_vec = p_back - p_front;



%% Target vector

target_vec = p_target - p_front;



%% Segment length

segment_norm = norm(segment_vec);



% Avoid division by zero

if segment_norm < 1e-6

    offset = 0;

    return

end



%% Projection

offset = dot(segment_vec,target_vec) / segment_norm;



end