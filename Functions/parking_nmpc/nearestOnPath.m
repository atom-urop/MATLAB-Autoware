function [s, ey] = nearestOnPath(ref, X, Y, s_guess)
%NEARESTONPATH  Where on the path is the point (X,Y)?
%   s  : distance along the path of the closest point
%   ey : how far to the side; positive = left of the path

idx = find(abs(ref.s - s_guess) < 1.5);          % only look nearby
idx = idx(idx < numel(ref.s));
if isempty(idx), idx = (1:numel(ref.s)-1).'; end

best = inf;  s = s_guess;  ey = 0;
for i = idx.'
    ax = ref.X(i);   ay = ref.Y(i);
    bx = ref.X(i+1); by = ref.Y(i+1);
    L2 = (bx-ax)^2 + (by-ay)^2;
    t  = max(0, min(1, ((X-ax)*(bx-ax) + (Y-ay)*(by-ay)) / max(L2, eps)));
    px = ax + t*(bx-ax);   py = ay + t*(by-ay);
    d2 = (X-px)^2 + (Y-py)^2;
    if d2 < best
        best = d2;
        s  = ref.s(i) + t*(ref.s(i+1) - ref.s(i));
        ey = sign((bx-ax)*(Y-ay) - (by-ay)*(X-ax)) * sqrt(d2);
    end
end
end