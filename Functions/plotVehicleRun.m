function plotVehicleRun(veh_log)
x = veh_log(:,1);  y = veh_log(:,2);
v = veh_log(:,3);  a = veh_log(:,4);

figure;
scatter(x, y, 20, v, 'filled');
colormap turbo; c = colorbar; c.Label.String = 'velocity [m/s]';
axis equal; grid on; xlabel('x [m]'); ylabel('y [m]');
title('Actual velocity along path');

figure;
scatter(x, y, 20, a, 'filled');
colormap turbo; c = colorbar; c.Label.String = 'acceleration [m/s^2]';
axis equal; grid on; xlabel('x [m]'); ylabel('y [m]');
title('Actual acceleration along path');
end