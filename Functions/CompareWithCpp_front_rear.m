function CompareWithCpp_front_rear()
% Compares the C++ velocity smoother output against ComputeSteeringRateAngle4WS.
% Both sides use the SAME curvature (taken from the C++ CSV), so any
% difference is attributable to the lookup, not to curvature calculation.

T = readtable('~/MATLAB-Autoware/Velocity_Smoother_Autoware/TestDataFromAutoware_front_rear.csv');
S = load('~/MATLAB-Autoware/LUT/LUT_rr_computation.mat');

% --- Run the MATLAB reference on the C++ curvature ---
traj = struct();
[~, out_m] = ComputeSteeringRateAngle4WS( ...
    traj, T.kappa_autoware, [], 0.1, ...
    S.k_ref_LUT, S.rr_LUT, S.delta_f_LUT);

front_m = out_m.front_wheel_angle_rad;
rear_m  = out_m.rear_wheel_angle_rad;

% --- Trajectory ---
figure; plot(T.x, T.y, '.-'); axis equal; grid on
xlabel('x [m]'); ylabel('y [m]');
title('90 deg left turn critical for 2WS');

% --- Steering angles, overlaid ---
figure; hold on; grid on
plot(T.i, T.front_rad, 'r-',  'LineWidth', 2);
plot(T.i, T.rear_rad,  'b-',  'LineWidth', 2);
plot(T.i, front_m,     'r--', 'LineWidth', 1);
plot(T.i, rear_m,      'b--', 'LineWidth', 1);
xlabel('trajectory point'); ylabel('steering angle [rad]');
legend('front C++','rear C++','front MATLAB','rear MATLAB','Location','best');
title('Front and rear steering: C++ vs MATLAB');

% --- Difference ---
figure; hold on; grid on
plot(T.i, T.front_rad - front_m, 'r');
plot(T.i, T.rear_rad  - rear_m,  'b');
xlabel('trajectory point'); ylabel('C++ minus MATLAB [rad]');
legend('front','rear'); title('Difference');

fprintf('max |front diff| = %.3e rad\n', max(abs(T.front_rad - front_m)));
fprintf('max |rear  diff| = %.3e rad\n', max(abs(T.rear_rad  - rear_m)));

% --- Curvature and applied ratio, for context ---
figure;
subplot(2,1,1); plot(T.i, T.kappa_autoware); grid on
ylabel('\kappa [1/m]'); title('Curvature computed by Autoware');
yline(max(S.k_ref_LUT), 'r--', 'LUT limit');
subplot(2,1,2); plot(T.i, T.rr_effective); grid on
ylabel('rear/front'); xlabel('trajectory point');
title('Effective steering ratio');
end