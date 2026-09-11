function drawOccupancyGrid(msg)
% Draws a nav_msgs/OccupancyGrid using ONLY the fields carried on the bus.

res = double(msg.info.resolution);
W   = double(msg.info.width);
H   = double(msg.info.height);
ox  = double(msg.info.origin.position.x);
oy  = double(msg.info.origin.position.y);
g   = double(msg.data);

x = ox + (0:W-1)*res;
y = oy + (0:H-1)*res;

figure('Name','costmap_generator / occupancy_grid');
imagesc(x, y, g, 'AlphaData', (g>=100)*0.55);
colormap(gca,[1 1 1; 0.85 0.25 0.25]);
set(gca,'YDir','normal','Layer','top');
axis equal; grid on; xlim([-16 16]); ylim([-14 14]);
xlabel('x [m]'); ylabel('y [m]');
title(sprintf('%dx%d @ %.2f m, origin (%.2f, %.2f)', W, H, res, ox, oy));
end