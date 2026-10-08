% function hit = collisionCheck(costmap, vehicle, x, y, theta)
% %COLLISIONCHECK  Can the car stand at pose (x,y,theta) without hitting anything?
% %
% % Reproduces, from autoware_freespace_planning_algorithms:
% %   AbstractPlanningAlgorithm::detectCollision     abstract_algorithm.cpp:618
% %   AbstractPlanningAlgorithm::detectBoundaryExit  abstract_algorithm.cpp:562
% % plus the 0.5 m vehicle inflation from
% %   FreespacePlannerNode::initializePlanningAlgorithm  freespace_planner_node.cpp:776
% %
% %   costmap : struct from CostMapGenerator
% %   vehicle : struct from vehicleParameters
% %   x, y    : base_link position [m]   (base_link = rear axle centre)
% %   theta   : heading [rad]
% %
% %   hit = true  ->  the body overlaps an obstacle, or leaves the map
% 
% p      = vehicle.param;
% margin = 0.5;                                     % vehicle_shape_margin_m
% 
% % --- the car's rectangle, measured from base_link -----------------------
% len   = p.front_overhang + p.wheel_base + p.rear_overhang + margin;
% wid   = p.wheel_tread + p.left_overhang + p.right_overhang + margin;
% back  = -(p.rear_overhang + margin/2);
% front = len + back;
% left  =  wid/2;
% right = -wid/2;
% 
% % --- place it at (x,y) facing theta, take its bounding box --------------
% c  = [front front back back ; left right right left];
% R  = [cos(theta) -sin(theta); sin(theta) cos(theta)];
% cw = R*c + [x; y];
% 
% res = costmap.res;  ox = costmap.origin(1);  oy = costmap.origin(2);
% W = double(costmap.width);  H = double(costmap.height);
% 
% ix0 = floor((min(cw(1,:))-ox)/res);  ix1 = ceil((max(cw(1,:))-ox)/res);
% iy0 = floor((min(cw(2,:))-oy)/res);  iy1 = ceil((max(cw(2,:))-oy)/res);
% 
% % --- leaving the map counts as a collision (detectBoundaryExit) ---------
% if ix0 < 0 || iy0 < 0 || ix1 > W-1 || iy1 > H-1
%     hit = true;  return;
% end
% 
% % % --- of the cells in that box, which are really under the car? ----------
% [IX, IY] = meshgrid(ix0:ix1, iy0:iy1);
% dx = ox + IX*res - x;   dy = oy + IY*res - y;
% bx =  cos(theta)*dx + sin(theta)*dy;              % into the car's own frame
% by = -sin(theta)*dx + cos(theta)*dy;
% under = bx >= back & bx <= front & by >= right & by <= left;
% 
% % --- which occupied CELLS touch the inflated car? -----------------------
% % A cell is a res x res square. Testing only its corner point can miss a
% % cell that reaches up to one cell width into the car. Test the cell
% % CENTRE against the car rectangle grown by half the cell diagonal: every
% % cell whose area touches the inflated car is then detected.
% % r   = res/sqrt(2);                                 % half cell diagonal [m]
% % ix0 = max(ix0-1, 0);  ix1 = min(ix1+1, W-1);       % look one cell further
% % iy0 = max(iy0-1, 0);  iy1 = min(iy1+1, H-1);
% % [IX, IY] = meshgrid(ix0:ix1, iy0:iy1);
% % dx = ox + (IX+0.5)*res - x;   dy = oy + (IY+0.5)*res - y;   % cell centres
% % bx =  cos(theta)*dx + sin(theta)*dy;              % into the car's own frame
% % by = -sin(theta)*dx + cos(theta)*dy;
% % under = bx >= back-r & bx <= front+r & by >= right-r & by <= left+r;
% 
% % --- is any of them occupied? -------------------------------------------
% % costmap.grid is 0/1; the ROS message carries 0/100. obstacle_threshold=100.
% cells = costmap.grid(sub2ind([H W], IY(under)+1, IX(under)+1));
% hit   = any(cells >= 1 | cells < 0);
% end



%==========================================================================
% function hit = collisionCheck(costmap, vehicle, x, y, theta)
% %COLLISIONCHECK
% % Return true if the enlarged vehicle footprint:
% %   1. leaves the costmap, or
% %   2. intersects the square area of an occupied/unknown costmap cell.
% %
% % The pose (x,y,theta) is the rear-axle-centre pose.
% %
% % IMPORTANT:
% % This project currently places grid-cell centres at:
% %
% %     centre_x = origin_x + column_index*resolution
% %     centre_y = origin_y + row_index*resolution
% %
% % This matches the existing costmap generators and imagesc plots.
% 
% %% 1. Enlarged vehicle footprint
% 
% p = vehicle.param;
% 
% % Total increase in vehicle length and width.
% % This gives 0.25 m around every side of the vehicle.
% margin = 0.5;
% 
% back = -(p.rear_overhang + margin/2);
% 
% front = ...
%     p.wheel_base + ...
%     p.front_overhang + ...
%     margin/2;
% 
% right = -( ...
%     p.wheel_tread + ...
%     p.left_overhang + ...
%     p.right_overhang + ...
%     margin)/2;
% 
% left = -right;
% 
% %% 2. Vehicle corners in world coordinates
% 
% ct = cos(theta);
% st = sin(theta);
% 
% R = [ct -st;
%      st  ct];
% 
% corners_body = [ ...
%     front, front, back, back;
%     left,  right, right, left];
% 
% corners_world = R*corners_body + [x; y];
% 
% vehicle_min_x = min(corners_world(1,:));
% vehicle_max_x = max(corners_world(1,:));
% vehicle_min_y = min(corners_world(2,:));
% vehicle_max_y = max(corners_world(2,:));
% 
% %% 3. Costmap information
% 
% res = double(costmap.res);
% 
% ox = double(costmap.origin(1));
% oy = double(costmap.origin(2));
% 
% W = double(costmap.width);
% H = double(costmap.height);
% 
% % Half-width of one square costmap cell.
% cell_half = res/2;
% 
% %% 4. Check whether the vehicle leaves the costmap
% 
% % In the current project convention, ox and oy are the centres of the
% % first grid cell. Therefore the map area extends half a cell beyond the
% % first and last cell centres.
% 
% map_min_x = ox - cell_half;
% map_max_x = ox + (W-1)*res + cell_half;
% 
% map_min_y = oy - cell_half;
% map_max_y = oy + (H-1)*res + cell_half;
% 
% if vehicle_min_x < map_min_x || ...
%    vehicle_max_x > map_max_x || ...
%    vehicle_min_y < map_min_y || ...
%    vehicle_max_y > map_max_y
% 
%     hit = true;
%     return;
% end
% 
% %% 5. Find costmap cells that could touch the vehicle
% 
% % Include every cell whose square area can reach the axis-aligned
% % bounding box of the rotated vehicle.
% 
% ix0 = ceil((vehicle_min_x - cell_half - ox)/res);
% ix1 = floor((vehicle_max_x + cell_half - ox)/res);
% 
% iy0 = ceil((vehicle_min_y - cell_half - oy)/res);
% iy1 = floor((vehicle_max_y + cell_half - oy)/res);
% 
% % Keep indices inside the costmap.
% ix0 = max(0,ix0);
% ix1 = min(W-1,ix1);
% 
% iy0 = max(0,iy0);
% iy1 = min(H-1,iy1);
% 
% if ix0 > ix1 || iy0 > iy1
%     hit = false;
%     return;
% end
% 
% %% 6. Keep only occupied or unknown cells
% 
% region = costmap.grid( ...
%     iy0+1:iy1+1, ...
%     ix0+1:ix1+1);
% 
% [local_row,local_col] = find( ...
%     region >= 1 | region < 0);
% 
% if isempty(local_row)
%     hit = false;
%     return;
% end
% 
% % Convert local region indices into complete costmap indices.
% cell_ix = ix0 + local_col - 1;
% cell_iy = iy0 + local_row - 1;
% 
% % Centres of the occupied cells.
% cell_x = ox + cell_ix*res;
% cell_y = oy + cell_iy*res;
% 
% %% 7. Represent the vehicle as an oriented rectangle
% 
% % The rear axle is not the geometric centre of the vehicle rectangle.
% vehicle_centre_offset = (front + back)/2;
% 
% vehicle_centre_x = x + ct*vehicle_centre_offset;
% vehicle_centre_y = y + st*vehicle_centre_offset;
% 
% vehicle_half_length = (front - back)/2;
% vehicle_half_width  = (left - right)/2;
% 
% % Vector from the vehicle rectangle centre to each occupied-cell centre.
% dx = cell_x - vehicle_centre_x;
% dy = cell_y - vehicle_centre_y;
% 
% %% 8. Exact rectangle-versus-square intersection using SAT
% 
% % SAT = Separating Axis Theorem.
% %
% % The vehicle and cell intersect only if their projections overlap on:
% %   1. world x-axis;
% %   2. world y-axis;
% %   3. vehicle longitudinal axis;
% %   4. vehicle lateral axis.
% 
% tolerance = 1e-12;
% 
% % Projection sizes on the world axes.
% vehicle_projection_x = ...
%     vehicle_half_length*abs(ct) + ...
%     vehicle_half_width*abs(st);
% 
% vehicle_projection_y = ...
%     vehicle_half_length*abs(st) + ...
%     vehicle_half_width*abs(ct);
% 
% overlap_world_x = ...
%     abs(dx) <= vehicle_projection_x + cell_half + tolerance;
% 
% overlap_world_y = ...
%     abs(dy) <= vehicle_projection_y + cell_half + tolerance;
% 
% % Cell-centre displacement expressed along the vehicle axes.
% distance_longitudinal = ct*dx + st*dy;
% distance_lateral      = -st*dx + ct*dy;
% 
% % Projection of an axis-aligned square cell onto either rotated vehicle
% % axis.
% cell_projection_rotated = ...
%     cell_half*(abs(ct) + abs(st));
% 
% overlap_vehicle_longitudinal = ...
%     abs(distance_longitudinal) <= ...
%     vehicle_half_length + cell_projection_rotated + tolerance;
% 
% overlap_vehicle_lateral = ...
%     abs(distance_lateral) <= ...
%     vehicle_half_width + cell_projection_rotated + tolerance;
% 
% % Intersection exists if all four projection tests overlap.
% cell_intersects_vehicle = ...
%     overlap_world_x & ...
%     overlap_world_y & ...
%     overlap_vehicle_longitudinal & ...
%     overlap_vehicle_lateral;
% 
% hit = any(cell_intersects_vehicle);
% 
% end


%==========================================================================
function hit = collisionCheck(costmap, vehicle, x, y, theta)
%COLLISIONCHECK Test the complete enlarged vehicle footprint.
% Return true if the footprint leaves the map or intersects the square area
% of an occupied or unknown costmap cell. The pose is at the rear axle.

%% Enlarged vehicle footprint

p = vehicle.param;

% The 0.5 m total enlargement gives 0.25 m clearance on every side.
margin = 0.5;

back = -(p.rear_overhang + margin/2);

front = ...
    p.wheel_base + ...
    p.front_overhang + ...
    margin/2;

right = -( ...
    p.wheel_tread + ...
    p.left_overhang + ...
    p.right_overhang + ...
    margin)/2;

left = -right;

%% Vehicle corners and axis-aligned bounds in world coordinates

ct = cos(theta);
st = sin(theta);

R = [ct -st;
     st  ct];

corners_body = [ ...
    front, front, back, back;
    left,  right, right, left];

corners_world = R*corners_body + [x; y];

vehicle_min_x = min(corners_world(1,:));
vehicle_max_x = max(corners_world(1,:));
vehicle_min_y = min(corners_world(2,:));
vehicle_max_y = max(corners_world(2,:));

%% Costmap geometry and boundary check

res = double(costmap.res);

ox = double(costmap.origin(1));
oy = double(costmap.origin(2));

W = double(costmap.width);
H = double(costmap.height);

cell_half = res/2;

% The costmap origin is the centre of the first grid cell.
map_min_x = ox - cell_half;
map_max_x = ox + (W-1)*res + cell_half;

map_min_y = oy - cell_half;
map_max_y = oy + (H-1)*res + cell_half;

if vehicle_min_x < map_min_x || ...
   vehicle_max_x > map_max_x || ...
   vehicle_min_y < map_min_y || ...
   vehicle_max_y > map_max_y

    hit = true;
    return;
end

%% Find occupied cells whose square area could touch the vehicle

ix0 = ceil((vehicle_min_x - cell_half - ox)/res);
ix1 = floor((vehicle_max_x + cell_half - ox)/res);

iy0 = ceil((vehicle_min_y - cell_half - oy)/res);
iy1 = floor((vehicle_max_y + cell_half - oy)/res);

ix0 = max(0,ix0);
ix1 = min(W-1,ix1);

iy0 = max(0,iy0);
iy1 = min(H-1,iy1);

if ix0 > ix1 || iy0 > iy1
    hit = false;
    return;
end

region = costmap.grid( ...
    iy0+1:iy1+1, ...
    ix0+1:ix1+1);

[local_row,local_col] = find( ...
    region >= 1 | region < 0);

if isempty(local_row)
    hit = false;
    return;
end

cell_ix = ix0 + local_col - 1;
cell_iy = iy0 + local_row - 1;

cell_x = ox + cell_ix*res;
cell_y = oy + cell_iy*res;

%% Exact oriented-rectangle versus square intersection using SAT

vehicle_centre_offset = (front + back)/2;

vehicle_centre_x = x + ct*vehicle_centre_offset;
vehicle_centre_y = y + st*vehicle_centre_offset;

vehicle_half_length = (front - back)/2;
vehicle_half_width  = (left - right)/2;

dx = cell_x - vehicle_centre_x;
dy = cell_y - vehicle_centre_y;

tolerance = 1e-12;

vehicle_projection_x = ...
    vehicle_half_length*abs(ct) + ...
    vehicle_half_width*abs(st);

vehicle_projection_y = ...
    vehicle_half_length*abs(st) + ...
    vehicle_half_width*abs(ct);

overlap_world_x = ...
    abs(dx) <= vehicle_projection_x + cell_half + tolerance;

overlap_world_y = ...
    abs(dy) <= vehicle_projection_y + cell_half + tolerance;

distance_longitudinal = ct*dx + st*dy;
distance_lateral      = -st*dx + ct*dy;

cell_projection_rotated = ...
    cell_half*(abs(ct) + abs(st));

overlap_vehicle_longitudinal = ...
    abs(distance_longitudinal) <= ...
    vehicle_half_length + cell_projection_rotated + tolerance;

overlap_vehicle_lateral = ...
    abs(distance_lateral) <= ...
    vehicle_half_width + cell_projection_rotated + tolerance;

cell_intersects_vehicle = ...
    overlap_world_x & ...
    overlap_world_y & ...
    overlap_vehicle_longitudinal & ...
    overlap_vehicle_lateral;

hit = any(cell_intersects_vehicle);
end