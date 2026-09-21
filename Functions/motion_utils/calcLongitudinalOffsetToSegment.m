function ego_offset_to_segment = calcLongitudinalOffsetToSegment( ...
    trajectory, ...
    seg_idx, ...
    ego_pose)


%% ============================================================
% CALCULATE LONGITUDINAL OFFSET TO SEGMENT
%
% Equivalent to:
%
% autoware::motion_utils::
% calcLongitudinalOffsetToSegment()
%
% Computes signed distance from trajectory point seg_idx
% to the orthogonal projection of ego position on the segment.
%
% ============================================================


N = length(trajectory.pose.position.x);



%% ============================================================
% Check segment index
% =============================================================

if seg_idx >= N

    error("Segment index out of range")

end


if seg_idx == N

    ego_offset_to_segment = NaN;
    return

end



%% ============================================================
% Segment points
%
% p_front -> trajectory(seg_idx)
% p_back  -> trajectory(seg_idx+1)
%
% =============================================================


p_front = [
    trajectory.pose.position.x(seg_idx);
    trajectory.pose.position.y(seg_idx);
    0
];


p_back = [
    trajectory.pose.position.x(seg_idx+1);
    trajectory.pose.position.y(seg_idx+1);
    0
];



%% ============================================================
% Ego point
% =============================================================


p_target = [
    ego_pose.pose.position.x;
    ego_pose.pose.position.y;
    0
];



%% ============================================================
% Segment vector
% =============================================================


segment_vec = p_back - p_front;



%% ============================================================
% Target vector
% =============================================================


target_vec = p_target - p_front;



%% ============================================================
% Projection length
%
% offset = segment_vec dot target_vec / |segment_vec|
%
% =============================================================


segment_norm = norm(segment_vec);



if segment_norm < 1e-6

    ego_offset_to_segment = NaN;
    return

end



ego_offset_to_segment = ...
    dot(segment_vec,target_vec) / segment_norm;



end