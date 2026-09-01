%% PlotVehicleFootprint.m
% Draws the vehicle body and all four wheels at their RECORDED angles,
% straight from the ROS 2 bag exports. Nothing here is illustrative:
% every pose and every wheel angle is measured data.
%
% Needs, in this folder:
%   2ws_odom.csv  2ws_front_steer.csv  2ws_rear_steer.csv
%   4ws_odom.csv  4ws_front_steer.csv  4ws_rear_steer.csv
%
% ATOM UROP - 4WS vehicle model

clear; close all; clc;

%% ATOM vehicle geometry (from the vehicle description package)
L      = 2.000;   % wheel_base [m]
TREAD  = 1.900;   % wheel_tread [m]
FO     = 0.460;   % front_overhang [m]
RO     = 0.460;   % rear_overhang [m]
WR     = 0.313;   % wheel_radius [m]
WW     = 0.235;   % wheel_width [m]

% base_link sits at the centre of the rear axle.

%% Snapshot times [s]
% 25 s falls inside the +0.3 rad phase, 53 s inside the -0.3 rad phase.
TIMES = [10 25 45 53];

runs = {'2ws','2WS baseline'; '4ws','4WS model'};

%% Figure 1 - pose gallery
figure('Name','Vehicle pose','Position',[60 60 1500 800],'Color','w');
tl = tiledlayout(2, numel(TIMES), 'TileSpacing','compact', 'Padding','compact');

for k = 1:2
    tag = runs{k,1};
    o = readtable([tag '_odom.csv']);
    f = readtable([tag '_front_steer.csv']);
    r = readtable([tag '_rear_steer.csv']);

    for j = 1:numel(TIMES)
        t = TIMES(j);
        [~, i] = min(abs(o.t - t));
        df = interp1(f.t, f.angle, t, 'linear', 0);
        dr = interp1(r.t, r.angle, t, 'linear', 0);

        nexttile((k-1)*numel(TIMES) + j); hold on;
        drawVehicle(o.x(i), o.y(i), o.yaw(i), df, dr, L, TREAD, FO, RO, WR, WW);
        axis equal;
        xlim(o.x(i) + [-3 3]); ylim(o.y(i) + [-3 3]);
        set(gca,'XTick',[],'YTick',[]); box on;
        title(sprintf('t = %d s   \\delta_f = %+.2f   \\delta_r = %+.2f rad', t, df, dr), ...
              'FontSize', 10);
        if j == 1
            ylabel(runs{k,2}, 'FontSize', 12, 'FontWeight','bold');
        end
    end
end
title(tl, 'Recorded vehicle pose   -   blue = front wheels, red = rear wheels', ...
      'FontSize', 14);

%% Figure 2 - side-by-side at the counter-steering instant
% The single clearest frame: rear wheels opposing the front.
T_CS = 53;
figure('Name','Counter-steering','Position',[60 60 1100 550],'Color','w');
for k = 1:2
    tag = runs{k,1};
    o = readtable([tag '_odom.csv']);
    f = readtable([tag '_front_steer.csv']);
    r = readtable([tag '_rear_steer.csv']);
    [~, i] = min(abs(o.t - T_CS));
    df = interp1(f.t, f.angle, T_CS, 'linear', 0);
    dr = interp1(r.t, r.angle, T_CS, 'linear', 0);

    subplot(1,2,k); hold on;
    drawVehicle(o.x(i), o.y(i), o.yaw(i), df, dr, L, TREAD, FO, RO, WR, WW);
    axis equal; xlim(o.x(i)+[-2.6 2.6]); ylim(o.y(i)+[-2.6 2.6]);
    set(gca,'XTick',[],'YTick',[]); box on;
    title(sprintf('%s\n\\delta_f = %+.2f, \\delta_r = %+.2f rad', runs{k,2}, df, dr), ...
          'FontSize', 12);
end

%% Figure 3 - crab angle
% beta = atan2(vy, vx): the angle between where the vehicle points and
% where it actually moves. Identically zero for a 2WS vehicle.
o2 = readtable('2ws_odom.csv');
o4 = readtable('4ws_odom.csv');
b2 = atan2(o2.vy, max(o2.vx, 1e-6));
b4 = atan2(o4.vy, max(o4.vx, 1e-6));

figure('Name','Crab angle','Position',[60 60 950 420],'Color','w');
plot(o2.t, rad2deg(b2), 'b-', 'LineWidth', 1.5); hold on;
plot(o4.t, rad2deg(b4), 'r-', 'LineWidth', 1.5);
grid on; xlabel('time [s]'); ylabel('\beta [deg]');
title('Crab angle - between heading and direction of travel');
legend('2WS','4WS','Location','best');

fprintf('Crab angle  2WS: %.4f .. %.4f deg\n', min(rad2deg(b2)), max(rad2deg(b2)));
fprintf('Crab angle  4WS: %.4f .. %.4f deg\n', min(rad2deg(b4)), max(rad2deg(b4)));

%% Optional animation
% Set to true to sweep through the 4WS run.
ANIMATE = false;
if ANIMATE
    o = readtable('4ws_odom.csv');
    f = readtable('4ws_front_steer.csv');
    r = readtable('4ws_rear_steer.csv');
    figure('Name','4WS animation','Position',[60 60 800 800],'Color','w');
    for t = 0:0.25:max(o.t)
        [~, i] = min(abs(o.t - t));
        df = interp1(f.t, f.angle, t, 'linear', 0);
        dr = interp1(r.t, r.angle, t, 'linear', 0);
        clf; hold on;
        plot(o.x, o.y, '-', 'Color', [.8 .8 .8]);
        drawVehicle(o.x(i), o.y(i), o.yaw(i), df, dr, L, TREAD, FO, RO, WR, WW);
        axis equal; xlim(o.x(i)+[-6 6]); ylim(o.y(i)+[-6 6]); grid on;
        title(sprintf('t = %.1f s   \\delta_r = %+.2f rad', t, dr));
        drawnow;
    end
end

%% ------------------------------------------------------------------
function drawVehicle(x, y, yaw, df, dr, L, TREAD, FO, RO, WR, WW)
% Body outline and four wheels, each wheel rotated by its own steer angle.
    R = @(th) [cos(th) -sin(th); sin(th) cos(th)];

    bw = TREAD + WW;
    body = [-RO -bw/2; L+FO -bw/2; L+FO bw/2; -RO bw/2; -RO -bw/2].';
    B = R(yaw)*body + [x; y];
    plot(B(1,:), B(2,:), 'k-', 'LineWidth', 1.6);

    % heading tick
    a = R(yaw)*[L/2 L/2+0.9; 0 0] + [x; y];
    plot(a(1,:), a(2,:), 'k-', 'LineWidth', 1.2);

    wheels = { 0, -TREAD/2, dr, [0.86 0.08 0.24];
               0,  TREAD/2, dr, [0.86 0.08 0.24];
               L, -TREAD/2, df, [0.00 0.45 0.74];
               L,  TREAD/2, df, [0.00 0.45 0.74] };

    for w = 1:4
        wx = wheels{w,1}; wy = wheels{w,2};
        ang = wheels{w,3}; col = wheels{w,4};
        rect = [-WR -WW/2; WR -WW/2; WR WW/2; -WR WW/2; -WR -WW/2].';
        P = R(yaw) * (R(ang)*rect + [wx; wy]) + [x; y];
        fill(P(1,:), P(2,:), col, 'EdgeColor','none');
    end
end