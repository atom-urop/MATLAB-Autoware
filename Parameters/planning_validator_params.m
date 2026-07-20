%% ============================================================
% AUTOWARE PLANNING VALIDATOR PARAMETERS
%
% Source:
% autoware_launch/config/planning/scenario_planning/common/
% planning_validator/trajectory_checker.param.yaml
%
%
% Parameters used by:
%
% planning_validator
%   └── TrajectoryChecker plugin
%
% to validate the generated trajectory before sending it to the controller.
%
%
% Adapted for MATLAB/Simulink 4WS simulation.
%
%
% Take care that from:
%
% /mnt/ssd/autoware/src/launcher/autoware_launch/autoware_launch/launch/
% components/tier4_planning_component.launch.xml
% 
% Parameters for planning validator are initialized, such as: 
%
%   planning_validator/trajectory_checker.param.yaml
%
% together with the common planning validator parameters.
%
%
% Here only trajectory_checker parameters are reported.
%
% Then, planning validator node is started from:
%
% /mnt/ssd/autoware/src/launcher/autoware_launch/tier4_universe_launch/
% tier4_planning_launch/launch/planning.launch.xml
%
%% ============================================================



%% ============================================================
% TRAJECTORY INTERVAL CHECK
%% ============================================================

planning_validator.interval.enable = true;

planning_validator.interval.threshold = 100.0;
% Maximum allowed distance between two consecutive trajectory points [m]



%% ============================================================
% CURVATURE CHECK
%% ============================================================

planning_validator.curvature.enable = true;

planning_validator.curvature.threshold = 1.0;
% Maximum allowed trajectory curvature [1/m]



%% ============================================================
% RELATIVE ANGLE CHECK
%% ============================================================

planning_validator.relative_angle.enable = true;

planning_validator.relative_angle.threshold = 2.0;
% Maximum allowed relative angle between consecutive trajectory segments [rad]



%% ============================================================
% LATERAL ACCELERATION CHECK
%% ============================================================

planning_validator.lateral_accel.enable = true;

planning_validator.lateral_accel.threshold = 9.8;
% Maximum allowed lateral acceleration [m/s^2]



%% ============================================================
% LONGITUDINAL ACCELERATION CHECK
%% ============================================================

planning_validator.min_lon_accel.enable = true;

planning_validator.min_lon_accel.threshold = -9.8;
% Minimum allowed longitudinal acceleration [m/s^2]


planning_validator.max_lon_accel.enable = true;

planning_validator.max_lon_accel.threshold = 9.8;
% Maximum allowed longitudinal acceleration [m/s^2]



%% ============================================================
% LATERAL JERK CHECK
%% ============================================================

planning_validator.lateral_jerk.enable = true;

planning_validator.lateral_jerk.threshold = 7.0;
% Maximum allowed lateral jerk [m/s^3]



%% ============================================================
% STEERING CHECK
%% ============================================================

planning_validator.steering.enable = true;

planning_validator.steering.threshold = 1.414;
% Maximum allowed steering angle [rad]
%
% In Autoware this threshold is evaluated assuming a 2WS vehicle:
%
%   steering = atan(curvature * wheel_base)
%
% For the 4WS implementation this check should be extended to evaluate
% both front and rear steering angles reconstructed from the trajectory
% curvature.



%% ============================================================
% STEERING RATE CHECK
%% ============================================================

planning_validator.steering_rate.enable = true;

planning_validator.steering_rate.threshold = 10.0;
% Maximum allowed steering rate [rad/s]
%
% In Autoware this threshold is evaluated from:
%
%   steer_rate =
%       (steer(i+1)-steer(i))/dt
%
% using steering reconstructed from the trajectory.
%
% For the 4WS implementation both front and rear steering rates should
% be computed, validating the worst-case value.



%% ============================================================
% DISTANCE DEVIATION CHECK
%% ============================================================

planning_validator.distance_deviation.enable = true;

planning_validator.distance_deviation.threshold = 100.0;
% Maximum lateral distance deviation [m]



%% ============================================================
% LONGITUDINAL DISTANCE DEVIATION CHECK
%% ============================================================

planning_validator.lon_distance_deviation.enable = true;

planning_validator.lon_distance_deviation.threshold = 1.0;
% Maximum longitudinal distance deviation [m]



%% ============================================================
% VELOCITY DEVIATION CHECK
%% ============================================================

planning_validator.velocity_deviation.enable = true;

planning_validator.velocity_deviation.threshold = 100.0;
% Maximum velocity deviation [m/s]



%% ============================================================
% YAW DEVIATION CHECK
%% ============================================================

planning_validator.yaw_deviation.enable = true;

planning_validator.yaw_deviation.threshold = 1.5708;
% Maximum yaw deviation [rad]


planning_validator.yaw_deviation.th_trajectory_yaw_shift = 0.1;
% Yaw validation is performed only if the nearest trajectory yaw changes
% more than this value [rad]



%% ============================================================
% FORWARD TRAJECTORY LENGTH CHECK
%% ============================================================

planning_validator.forward_trajectory_length.enable = true;

planning_validator.forward_trajectory_length.acceleration = -3.0;
% Assumed braking acceleration [m/s^2]


planning_validator.forward_trajectory_length.margin = 2.0;
% Safety margin added to stopping distance [m]



%% ============================================================
% TRAJECTORY SHIFT CHECK
%% ============================================================

planning_validator.trajectory_shift.enable = true;

planning_validator.trajectory_shift.lat_shift_th = 0.5;
% Maximum allowed lateral trajectory shift [m]


planning_validator.trajectory_shift.forward_shift_th = 1.0;
% Maximum allowed forward trajectory shift [m]


planning_validator.trajectory_shift.backward_shift_th = 0.1;
% Maximum allowed backward trajectory shift [m]


planning_validator.trajectory_shift.handling_type = 2;
% Diagnostic handling type


planning_validator.trajectory_shift.override_error_diag = false;
% Override diagnostic error state



%% ============================================================
% CREATE VECTOR FOR MASK / FROM WORKSPACE IF REQUIRED
%% ============================================================

planning_validator_row = [ ...
    planning_validator.curvature.threshold,...
    planning_validator.steering.threshold,...
    planning_validator.steering_rate.threshold,...
    planning_validator.lateral_accel.threshold,...
    planning_validator.lateral_jerk.threshold ...
    ];

planning_validator_ws.time = 0;
planning_validator_ws.signals.values = planning_validator_row;
planning_validator_ws.signals.dimensions = length(planning_validator_row);