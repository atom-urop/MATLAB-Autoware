function [x,y,z,qx,qy,qz,qw,vx,vy,ax,wz,df,dr] = pathToTrajectory(path, seg, v_mps)
%PATHTOTRAJECTORY  One gear-segment of the plan, resampled to N = 100 points.
%
% Mirrors the partial-trajectory split in autoware_freespace_planner/utils.cpp:
% the plan is cut at every forward/reverse change and published one piece
% at a time, because a controller cannot track a path that reverses.
%
% path  : [M x 4] from the planner, columns [x y theta is_back]
% seg   : which segment to emit (1 = the first)
% v_mps : placeholder speed; velocity_smoother overwrites it anyway

N = 100;
% if size(path,1) >= 2, path(1,4) = path(2,4); end   % start node inherits the first move's direction

if size(path,1) >= 2
    nc = min(6, size(path,2));
    path(1,4:nc) = path(2,4:nc);   % start inherits the first move's direction AND steering
end

if nargin < 3, v_mps = 8/3.6; end          % same 8 km/h as the sinusoid block

% cut at every direction change
% Find the first point reached using each new direction.
direction_change_points = find(diff(path(:,4)) ~= 0) + 1;

% Consecutive segments share the position immediately before each
% direction change. This position is where the vehicle stops and reverses.
segment_starts = [1; direction_change_points - 1];
segment_ends   = [direction_change_points - 1; size(path,1)];

P = path(segment_starts(seg):segment_ends(seg), :);

% For every segment after a direction change, the shared first position
% inherits the direction and steering of the new segment.
if seg > 1 && size(P,1) >= 2
    nc = min(6, size(P,2));
    P(1,4:nc) = P(2,4:nc);
end

if size(P,1) < 2
    error('pathToTrajectory: segment %d has fewer than 2 points', seg);
end

% resample to exactly N points, evenly spaced along the path
s  = [0; cumsum(hypot(diff(P(:,1)), diff(P(:,2))))];
sq = linspace(0, s(end), N)';
x   = interp1(s, P(:,1), sq);
y   = interp1(s, P(:,2), sq);
yaw = interp1(s, unwrap(P(:,3)), sq);

z  = zeros(N,1);
qx = zeros(N,1);   qy = zeros(N,1);
qz = sin(yaw/2);   qw = cos(yaw/2);        % same convention as the sinusoid block

vx = v_mps*ones(N,1);   if P(1,4) == 1, vx = -vx; end   % reverse segment
vy = zeros(N,1);   ax = zeros(N,1);   wz = zeros(N,1);
% df = zeros(N,1);   dr = zeros(N,1);        % rear angle: the 4WS slot, empty for now
if size(P,2) >= 6
    % 'previous' holds each command constant over its arc. The planner
    % commanded discrete steering steps, not a smooth sweep between them.
    df = interp1(s, P(:,5), sq, 'next');   % stretch k -> k+1 uses the steering stored in point k+1
    dr = interp1(s, P(:,6), sq, 'next');
else
    df = zeros(N,1);   dr = zeros(N,1);
end
end