function Plot2WSvs4WS()
T = readtable('/home/husain5/MATLAB-Autoware/TestDataFromAutoware_front_rear_velocity.csv');

% --- 1. Trajectory, coloured by curvature ---
figure;
scatter(T.x, T.y, 25, T.kappa, 'filled'); axis equal; grid on
c = colorbar; c.Label.String = '\kappa [1/m]';
xlabel('x [m]'); ylabel('y [m]');
title('90 deg left turn (R = 1 m)');


% --- 2. Front steering: the core claim ---
S = load('/home/husain5/MATLAB-Autoware/LUT/LUT_rr_computation.mat');
L = 2.0;

front_2ws_m = atan(L * T.kappa);
rear_2ws_m  = zeros(size(T.kappa));

traj = struct();
[~, out_m] = ComputeSteeringRateAngle4WS(traj, T.kappa, [], 0.1, ...
    S.k_ref_LUT, S.rr_LUT, S.delta_f_LUT);
front_4ws_m = out_m.front_wheel_angle_rad;
rear_4ws_m  = out_m.rear_wheel_angle_rad;

% Colour scheme: warm = 2WS, cool = 4WS. Bright = front, pale = rear.
c2f = [0.90 0.20 0.15];   % 2WS front  - red
c2r = [1.00 0.65 0.10];   % 2WS rear   - orange
c4f = [0.15 0.55 0.95];   % 4WS front  - blue
c4r = [0.20 0.85 0.55];   % 4WS rear   - green

figure; hold on; grid on; box on

plot(T.i, T.front_2ws, '-',  'Color', c2f, 'LineWidth', 2.0);
plot(T.i, front_2ws_m, '--', 'Color', [1 1 1]*0.95, 'LineWidth', 1.0);

plot(T.i, T.front_4ws, '-',  'Color', c4f, 'LineWidth', 2.0);
plot(T.i, front_4ws_m, '--', 'Color', [1 1 1]*0.95, 'LineWidth', 1.0);

plot(T.i, T.rear_2ws,  '-',  'Color', c2r, 'LineWidth', 2.0);
plot(T.i, rear_2ws_m,  '--', 'Color', [1 1 1]*0.95, 'LineWidth', 1.0);

plot(T.i, T.rear_4ws,  '-',  'Color', c4r, 'LineWidth', 2.0);
plot(T.i, rear_4ws_m,  '--', 'Color', [1 1 1]*0.95, 'LineWidth', 1.0);

yline( 0.7, ':', 'Color', [0.7 0.7 0.7]);
yline(-0.7, ':', 'Color', [0.7 0.7 0.7]);

xlabel('trajectory point'); ylabel('steering angle [rad]');
legend({'front 2WS (C++)','front 2WS (MATLAB)', ...
        'front 4WS (C++)','front 4WS (MATLAB)', ...
        'rear 2WS (C++)','rear 2WS (MATLAB)', ...
        'rear 4WS (C++)','rear 4WS (MATLAB)'}, ...
        'Location','eastoutside');
title('Front and rear steering: 2WS vs 4WS, C++ vs MATLAB');
ylim([-0.85 1.25]);

fprintf('max |front 2WS diff| = %.3e rad\n', max(abs(T.front_2ws - front_2ws_m)));
fprintf('max |front 4WS diff| = %.3e rad\n', max(abs(T.front_4ws - front_4ws_m)));
fprintf('max |rear  4WS diff| = %.3e rad\n', max(abs(T.rear_4ws  - rear_4ws_m)));


% --- 3. How much front steering 4WS saves ---
figure; hold on; grid on
plot(T.i, T.d_front, 'r', 'LineWidth', 1.5);
xlabel('trajectory point'); ylabel('front_{4WS} - front_{2WS} [rad]');
title('Front steering reduction from 4WS');


% --- 4. Velocity ---
figure;
subplot(2,1,1); hold on; grid on
plot(T.i, T.vel_2ws, 'r-', 'LineWidth', 2);
plot(T.i, T.vel_4ws, 'b-', 'LineWidth', 1.5);
ylabel('velocity [m/s]'); legend('2WS','4WS','Location','best');
title('Planned velocity');
subplot(2,1,2); grid on
plot(T.i, T.vel_ratio, 'g'); yline(1, 'k:');
ylabel('v_{4WS} / v_{2WS}'); xlabel('trajectory point');


% --- Summary numbers ---
fprintf('\nmax front 2WS      : %.4f rad\n', max(abs(T.front_2ws)));
fprintf('max front 4WS      : %.4f rad\n', max(abs(T.front_4ws)));
fprintf('max rear  4WS      : %.4f rad\n', max(abs(T.rear_4ws)));
fprintf('largest reduction  : %.4f rad\n', max(abs(T.d_front)));
fprintf('min velocity ratio : %.4f\n', min(T.vel_ratio));
fprintf('rows at saturation : %d of %d\n', sum(abs(T.front_4ws) > 0.699), height(T));
end