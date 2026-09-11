function costmap = CostMapGenerator(scene, vehicle)
p = vehicle.param;

res   = 0.3;  len_x = 70.0;  len_y = 70.0;
W = ceil(len_x/res);  H = ceil(len_y/res);
origin_x = -(W/2)*res;  origin_y = -(H/2)*res;
[IX, IY] = meshgrid(0:W-1, 0:H-1);
X = origin_x + IX*res;  Y = origin_y + IY*res;

car_len = p.front_overhang + p.wheel_base + p.rear_overhang;
car_wid = p.wheel_tread + p.left_overhang + p.right_overhang;
expand  = 0.5;

switch scene
    case 'perpendicular_lot'
        lot = [-15 -12; 15 -12; 15 12; -15 12];
        bay_x = (-3:3)*3.6;  bay_y = 8.75;  empty_bay = 4;
    case 'empty_lot'
        lot = [-15 -12; 15 -12; 15 12; -15 12];
        bay_x = [];  bay_y = 0;  empty_bay = 0;
    case 'narrow_corridor'
        lot = [-4 -12; 4 -12; 4 12; -4 12];
        bay_x = [];  bay_y = 0;  empty_bay = 0;
    otherwise
        error('makeCostmap: unknown scene "%s"', scene);
end

occ = ones(H,W);
occ(inpolygon(X,Y,lot(:,1),lot(:,2))) = 0;

L = car_len + expand;  D = car_wid + expand;
corners = [ L/2 L/2 -L/2 -L/2; D/2 -D/2 -D/2 D/2 ];
Rz = [0 -1; 1 0];
for k = 1:numel(bay_x)
    if k == empty_bay, continue; end
    q = Rz*corners + [bay_x(k); bay_y];
    occ(inpolygon(X,Y,q(1,:),q(2,:))) = 1;
end

costmap.scene  = scene;
costmap.res    = res;
costmap.width  = W;
costmap.height = H;
costmap.origin = [origin_x origin_y];
costmap.grid   = occ;

costmap.X = X;  costmap.Y = Y;
costmap.start_pose = [0, 3.0, pi/2];
if empty_bay > 0
    costmap.goal_pose = [bay_x(empty_bay), ...
        bay_y - (p.wheel_base + p.front_overhang - p.rear_overhang)/2, pi/2];
else
    costmap.goal_pose = [0, 10.0, pi/2];
end
% ---- Simulink bus payload: nav_msgs/OccupancyGrid ----
costmap.msg.info.resolution           = res;
costmap.msg.info.width                = uint32(W);
costmap.msg.info.height               = uint32(H);
costmap.msg.info.origin.position.x    = origin_x;
costmap.msg.info.origin.position.y    = origin_y;
costmap.msg.info.origin.position.z    = 0;
costmap.msg.info.origin.orientation.x = 0;
costmap.msg.info.origin.orientation.y = 0;
costmap.msg.info.origin.orientation.z = 0;
costmap.msg.info.origin.orientation.w = 1;
costmap.msg.data                      = int8(occ*100);
end