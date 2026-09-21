function PlotProofOfLiveTopicFromPlanningSimulator()
% One captured message from
% /planning/scenario_planning/velocity_smoother/trajectory
% on the running Autoware stack.

T = readtable('/home/husain5/MATLAB-Autoware/proof.csv');

C_F = [0.11 0.42 0.76];   % front
C_R = [0.13 0.60 0.35];   % rear
C_V = [0.75 0.22 0.17];   % velocity
GR  = [0.55 0.55 0.55];

% ---------- Figure 1: path coloured by velocity ----------
figure('Color','w','Position',[100 100 760 640]);
hold on; grid on; box on
plot(T.x, T.y, '-', 'Color',[0.80 0.80 0.80], 'LineWidth',1.0);
scatter(T.x, T.y, 26, T.longitudinal_velocity_mps, 'filled');
plot(T.x(1), T.y(1), 'o', 'MarkerSize',8, 'MarkerFaceColor','w', ...
     'MarkerEdgeColor','k', 'LineWidth',1.4);
text(T.x(1), T.y(1), '  start', 'FontSize',9, 'VerticalAlignment','bottom');
axis equal
colormap(turbo);
cb = colorbar; cb.Label.String = 'longitudinal velocity  [m/s]';
xlabel('x  [m]'); ylabel('y  [m]');
title('Planned trajectory, coloured by velocity');
set(gca,'FontSize',10);

% ---------- Figure 2: steering angles ----------
figure('Color','w','Position',[140 140 900 460]);
hold on; grid on; box on
plot(T.i, T.front_wheel_angle_rad, '-', 'Color',C_F, 'LineWidth',2.0);
plot(T.i, T.rear_wheel_angle_rad,  '-', 'Color',C_R, 'LineWidth',2.0);
yline(0,'-','Color',GR,'HandleVisibility','off');
xlabel('trajectory point'); ylabel('steering angle  [rad]');
legend({'front','rear'},'Location','best','FontSize',10);
nz = sum(abs(T.rear_wheel_angle_rad) > 1e-6);
title(sprintf('Front and rear steering  (rear non-zero at %d of %d points)', ...
      nz, height(T)));
xlim([0 max(T.i)]);
set(gca,'FontSize',10);

% ---------- Figure 3: velocity ----------
figure('Color','w','Position',[180 180 900 460]);
hold on; grid on; box on
plot(T.i, T.longitudinal_velocity_mps, '-', 'Color',C_V, 'LineWidth',2.0);
yline(2.0,'--','Color',GR,'HandleVisibility','off');
text(2, 2.06, 'min\_curve\_velocity', 'FontSize',9, 'Color',GR);
xlabel('trajectory point'); ylabel('velocity  [m/s]');
title('Planned longitudinal velocity');
xlim([0 max(T.i)]);
set(gca,'FontSize',10);

fprintf('\npoints            : %d\n', height(T));
fprintf('front range       : %+.4f to %+.4f rad\n', ...
        min(T.front_wheel_angle_rad), max(T.front_wheel_angle_rad));
fprintf('rear  range       : %+.4f to %+.4f rad\n', ...
        min(T.rear_wheel_angle_rad), max(T.rear_wheel_angle_rad));
fprintf('rear non-zero     : %d of %d points\n', nz, height(T));
fprintf('velocity range    : %.3f to %.3f m/s\n', ...
        min(T.longitudinal_velocity_mps), max(T.longitudinal_velocity_mps));
end