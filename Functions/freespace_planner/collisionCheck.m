function hit = collisionCheck(costmap, vehicle, x, y, theta)
%COLLISIONCHECK  Can the car stand at pose (x,y,theta) without hitting anything?
%
% Reproduces, from autoware_freespace_planning_algorithms:
%   AbstractPlanningAlgorithm::detectCollision     abstract_algorithm.cpp:618
%   AbstractPlanningAlgorithm::detectBoundaryExit  abstract_algorithm.cpp:562
% plus the 0.5 m vehicle inflation from
%   FreespacePlannerNode::initializePlanningAlgorithm  freespace_planner_node.cpp:776
%
%   costmap : struct from CostMapGenerator
%   vehicle : struct from vehicleParameters
%   x, y    : base_link position [m]   (base_link = rear axle centre)
%   theta   : heading [rad]
%
%   hit = true  ->  the body overlaps an obstacle, or leaves the map

p      = vehicle.param;
margin = 0.5;                                     % vehicle_shape_margin_m

% --- the car's rectangle, measured from base_link -----------------------
len   = p.front_overhang + p.wheel_base + p.rear_overhang + margin;
wid   = p.wheel_tread + p.left_overhang + p.right_overhang + margin;
back  = -(p.rear_overhang + margin/2);
front = len + back;
left  =  wid/2;
right = -wid/2;

% --- place it at (x,y) facing theta, take its bounding box --------------
c  = [front front back back ; left right right left];
R  = [cos(theta) -sin(theta); sin(theta) cos(theta)];
cw = R*c + [x; y];

res = costmap.res;  ox = costmap.origin(1);  oy = costmap.origin(2);
W = double(costmap.width);  H = double(costmap.height);

ix0 = floor((min(cw(1,:))-ox)/res);  ix1 = ceil((max(cw(1,:))-ox)/res);
iy0 = floor((min(cw(2,:))-oy)/res);  iy1 = ceil((max(cw(2,:))-oy)/res);

% --- leaving the map counts as a collision (detectBoundaryExit) ---------
if ix0 < 0 || iy0 < 0 || ix1 > W-1 || iy1 > H-1
    hit = true;  return;
end

% % --- of the cells in that box, which are really under the car? ----------
[IX, IY] = meshgrid(ix0:ix1, iy0:iy1);
dx = ox + IX*res - x;   dy = oy + IY*res - y;
bx =  cos(theta)*dx + sin(theta)*dy;              % into the car's own frame
by = -sin(theta)*dx + cos(theta)*dy;
under = bx >= back & bx <= front & by >= right & by <= left;

% --- which occupied CELLS touch the inflated car? -----------------------
% A cell is a res x res square. Testing only its corner point can miss a
% cell that reaches up to one cell width into the car. Test the cell
% CENTRE against the car rectangle grown by half the cell diagonal: every
% cell whose area touches the inflated car is then detected.
% r   = res/sqrt(2);                                 % half cell diagonal [m]
% ix0 = max(ix0-1, 0);  ix1 = min(ix1+1, W-1);       % look one cell further
% iy0 = max(iy0-1, 0);  iy1 = min(iy1+1, H-1);
% [IX, IY] = meshgrid(ix0:ix1, iy0:iy1);
% dx = ox + (IX+0.5)*res - x;   dy = oy + (IY+0.5)*res - y;   % cell centres
% bx =  cos(theta)*dx + sin(theta)*dy;              % into the car's own frame
% by = -sin(theta)*dx + cos(theta)*dy;
% under = bx >= back-r & bx <= front+r & by >= right-r & by <= left+r;

% --- is any of them occupied? -------------------------------------------
% costmap.grid is 0/1; the ROS message carries 0/100. obstacle_threshold=100.
cells = costmap.grid(sub2ind([H W], IY(under)+1, IX(under)+1));
hit   = any(cells >= 1 | cells < 0);
end