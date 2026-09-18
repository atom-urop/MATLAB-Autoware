function d = vehicleClearance(costmap, vehicle, x, y, th, d_preferred)

%% Just created in order to consider the clearance between the obstacle and the vehicle into the cost function in the hybridAstar.m


% Clearance outside the same enlarged footprint as collisionCheck.
% The result is capped at d_preferred: larger distances cost nothing.

p = vehicle.param;
margin = 0.5;  % Must match collisionCheck

back  = -(p.rear_overhang + margin/2);
front = p.wheel_base + p.front_overhang + margin/2;
half_width = (p.wheel_tread + p.left_overhang + ...
    p.right_overhang + margin)/2;

% Vehicle corners in map coordinates
corners = [front front back back; ...
    half_width -half_width -half_width half_width];

R = [cos(th) -sin(th); sin(th) cos(th)];
corners = R*corners + [x; y];

res = costmap.res;
ox = costmap.origin(1);
oy = costmap.origin(2);
W = double(costmap.width);
H = double(costmap.height);

% Include distance to the map limits used by collisionCheck
d = max(0, min([d_preferred, ...
    min(corners(1,:)) - ox, ...
    ox + (W-1)*res - max(corners(1,:)), ...
    min(corners(2,:)) - oy, ...
    oy + (H-1)*res - max(corners(2,:))]));

if d == 0
    return;
end

% Only inspect nearby map cells
cell_radius = res/sqrt(2);
padding = d_preferred + cell_radius;

ix0 = max(0, floor((min(corners(1,:))-padding-ox)/res));
ix1 = min(W-1, ceil((max(corners(1,:))+padding-ox)/res));
iy0 = max(0, floor((min(corners(2,:))-padding-oy)/res));
iy1 = min(H-1, ceil((max(corners(2,:))+padding-oy)/res));

region = costmap.grid(iy0+1:iy1+1, ix0+1:ix1+1);
[row, col] = find(region >= 1 | region < 0);

if isempty(row)
    return;
end

% Occupied-cell centres expressed in the vehicle frame
dx = ox + (ix0 + col - 1)*res - x;
dy = oy + (iy0 + row - 1)*res - y;

bx =  cos(th)*dx + sin(th)*dy;
by = -sin(th)*dx + cos(th)*dy;

% Distance from each occupied-cell centre to the rectangle
gap_x = max(max(back-bx, bx-front), 0);
gap_y = max(abs(by)-half_width, 0);

% Account conservatively for the occupied cells' area
obstacle_clearance = max(0, ...
    min(hypot(gap_x, gap_y)) - cell_radius);

d = min(d, obstacle_clearance);
end