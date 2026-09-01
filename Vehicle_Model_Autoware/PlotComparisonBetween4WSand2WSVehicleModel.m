%% PlotBagComparison.m
% Compares the 2WS baseline against the 4WS run, from data exported from
% ROS 2 bags recorded on the Jetson.
% We have recorded two ros2 bags: 2ws and 4WS.
% For the first bag 2ws: We have launched the simple planning simulator and
% we did not sent any message to the rear steer input of the vehicle model
% (the lateral control module still do not send anything to the rear (we
% have not modified it yet
% For the second bag 4WS: We have launched the simulation again, but now at
% two arbitrary points while the car is moving we set at one the rear steer
% to be 0.3 and then at another point to be -0.3. This is done as a sort of
% checking if the vehicle model can hold rear steer angle and how it will
% react

clear; close all; clc;

%% Load
o2 = readtable('~/MATLAB-Autoware/Vehicle_Model_Autoware/2ws_odom.csv');
r2 = readtable('~/MATLAB-Autoware/Vehicle_Model_Autoware/2ws_rear_steer.csv');
f2 = readtable('~/MATLAB-Autoware/Vehicle_Model_Autoware/2ws_front_steer.csv');

o4 = readtable('~/MATLAB-Autoware/Vehicle_Model_Autoware/4ws_odom.csv');
r4 = readtable('~/MATLAB-Autoware/Vehicle_Model_Autoware/4ws_rear_steer.csv');
f4 = readtable('~/MATLAB-Autoware/Vehicle_Model_Autoware/4ws_front_steer.csv');

%% Figure 1 - Trajectory overlay
figure('Name','Trajectory','Position',[100 100 700 600]);
plot(o2.x, o2.y, 'b-',  'LineWidth', 1.6); hold on;
plot(o4.x, o4.y, 'r-',  'LineWidth', 1.6);
plot(o2.x(1), o2.y(1), 'ko', 'MarkerFaceColor','k', 'MarkerSize', 8);
axis equal; grid on;
xlabel('x [m]'); ylabel('y [m]');
title('Vehicle trajectory: 2WS baseline vs 4WS');
legend('2WS (no rear command)','4WS (rear commanded)','Start','Location','best');

%% Figure 2 - Lateral velocity
% v_y = v_x * tan(delta_r). Identically zero for a 2WS vehicle.
figure('Name','Lateral velocity','Position',[100 100 900 400]);
plot(o2.t, o2.vy, 'b-', 'LineWidth', 1.4); hold on;
plot(o4.t, o4.vy, 'r-', 'LineWidth', 1.4);
grid on; xlabel('time [s]'); ylabel('v_y [m/s]');
title('Lateral velocity - zero by construction for 2WS');
legend('2WS','4WS','Location','best');

%% Figure 3 - Rear steering angle
figure('Name','Rear steering','Position',[100 100 900 400]);
plot(r2.t, r2.angle, 'b-', 'LineWidth', 1.4); hold on;
plot(r4.t, r4.angle, 'r-', 'LineWidth', 1.4);
grid on; xlabel('time [s]'); ylabel('\delta_r [rad]');
title('Rear steering angle');
legend('2WS','4WS','Location','best');

%% Figure 4 - Front steering angle (controller response)
figure('Name','Front steering','Position',[100 100 900 400]);
plot(f2.t, f2.angle, 'b-', 'LineWidth', 1.2); hold on;
plot(f4.t, f4.angle, 'r-', 'LineWidth', 1.2);
grid on; xlabel('time [s]'); ylabel('\delta_f [rad]');
title('Front steering angle - controller reacting to the rear-axle disturbance');
legend('2WS','4WS','Location','best');

%% Figure 5 - Yaw rate
figure('Name','Yaw rate','Position',[100 100 900 400]);
plot(o2.t, o2.wz, 'b-', 'LineWidth', 1.4); hold on;
plot(o4.t, o4.wz, 'r-', 'LineWidth', 1.4);
grid on; xlabel('time [s]'); ylabel('\omega_z [rad/s]');
title('Yaw rate');
legend('2WS','4WS','Location','best');

%% Physics cross-check
% The model states v_y = v_x * tan(delta_r). Verify the recorded data
% obeys it, by interpolating the rear angle onto the odometry timebase.
dr_i     = interp1(r4.t, r4.angle, o4.t, 'linear', 0);
vy_model = o4.vx .* tan(dr_i);
err      = o4.vy - vy_model;

fprintf('--- v_y = v_x * tan(delta_r) check ---\n');
fprintf('  max |error|  : %.3e m/s\n', max(abs(err)));
fprintf('  RMS  error   : %.3e m/s\n', rms(err));

figure('Name','Model check','Position',[100 100 900 400]);
plot(o4.t, o4.vy,   'r-',  'LineWidth', 1.6); hold on;
plot(o4.t, vy_model,'k--', 'LineWidth', 1.2);
grid on; xlabel('time [s]'); ylabel('v_y [m/s]');
title('Recorded v_y vs v_x\cdottan(\delta_r)');
legend('recorded','model','Location','best');

%% Summary
fprintf('\n--- Summary ---\n');
fprintf('              2WS            4WS\n');
fprintf('vy range   [%6.3f %6.3f]  [%6.3f %6.3f] m/s\n', ...
    min(o2.vy), max(o2.vy), min(o4.vy), max(o4.vy));
fprintf('wz range   [%6.3f %6.3f]  [%6.3f %6.3f] rad/s\n', ...
    min(o2.wz), max(o2.wz), min(o4.wz), max(o4.wz));
fprintf('dr range   [%6.3f %6.3f]  [%6.3f %6.3f] rad\n', ...
    min(r2.angle), max(r2.angle), min(r4.angle), max(r4.angle));