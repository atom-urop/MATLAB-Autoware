%% ============================================================
% AUTOWARE VELOCITY SMOOTHER PARAMETERS
%
% Source:
% autoware_launch/config/planning/scenario_planning/common/
% autoware_velocity_smoother/velocity_smoother.param.yaml
%
%
% Parameters used for:
% - curvature calculation
% - lateral acceleration limit
% - steering rate limit
%
% Adapted for MATLAB/Simulink 4WS simulation
% 
% 
% 
% % Take care that from /mnt/ssd/autoware/src/launcher/autoware_launch/
% tier4_universe_launch/tier4_planning_launch/launch/scenario_planning/scenario_planning.launch.xml
% 
% the velocity_smoother node is initialized, importing all the parameters
% (collected in node_params_ and base_params_ in node.cpp). In particular
% parameters are coming from:
%   - velocity_smoother_path_param_type: /mnt/ssd/autoware/src/launcher/autoware_launch/autoware_launch/config/planning/scenario_planning/common/autoware_velocity_smoother/JerkFiltered.param.yaml
%   - common_param_path: /mnt/ssd/autoware/src/launcher/autoware_launch/autoware_launch/config/planning/scenario_planning/common/common.param.yaml
%   - velocity_smoother_param_path: /mnt/ssd/autoware/src/launcher/autoware_launch/autoware_launch/config/planning/scenario_planning/common/autoware_velocity_smoother/velocity_smoother.param.yaml
%   - nearest_search_param_path: /mnt/ssd/autoware/src/launcher/autoware_launch/autoware_launch/config/planning/scenario_planning/common/nearest_search.param.yaml
% 
% Here just velocity_smoother_param_path parameters are reported.
% % ============================================================


%% ============================================================
% CURVATURE PARAMETERS
% =============================================================

velocity_smoother.curvature_calculation_distance = 2.0;
% Distance between trajectory points used for curvature calculation [m]


velocity_smoother.curvature_threshold = 0.02;
% Curvature threshold triggering steering rate limitation [1/m]


%% ============================================================
% RESAMPLING PARAMETERS
% =============================================================

velocity_smoother.resample_ds = 0.1;
% Distance between trajectory points after resampling [m]


velocity_smoother.input_points_interval = 1.0;
% Original trajectory point spacing used when resampling is disabled [m]


%% ============================================================
% LATERAL ACCELERATION LIMIT
% =============================================================

velocity_smoother.enable_lateral_acc_limit = true;


velocity_smoother.lateral_acceleration_limits = [
    1.0
    1.0
    1.0
    1.0
];
% Maximum lateral acceleration limits [m/s^2]


velocity_smoother.min_curve_velocity = 2.0;
% Minimum velocity allowed in curves [m/s]


velocity_smoother.decel_distance_before_curve = 3.5;
% Distance before curve where deceleration starts [m]


velocity_smoother.decel_distance_after_curve = 2.0;
% Distance after curve where velocity adaptation continues [m]


velocity_smoother.min_decel_for_lateral_acc_lim_filter = -2.5;
% Maximum allowed deceleration caused by lateral acceleration filter [m/s^2]



%% ============================================================
% STEERING RATE LIMIT PARAMETERS
% =============================================================

velocity_smoother.enable_steering_rate_limit = true;


velocity_smoother.velocity_thresholds = [
    0.1
    0.3
    20.0
    30.0
];
% Velocity thresholds for steering rate interpolation [m/s]


velocity_smoother.steering_angle_rate_limits = [
    11.5
    11.5
    10.5
    3.5
];
% Steering angle rate limits [deg/s]


%% ============================================================
% ENGAGE / REPLAN PARAMETERS
% =============================================================

velocity_smoother.replan_vel_deviation = 5.53;
% Velocity deviation triggering replanning [m/s]


velocity_smoother.engage_velocity = 0.25;
% Engage velocity threshold [m/s]


velocity_smoother.engage_acceleration = 0.5;
% Engage acceleration [m/s^2]


velocity_smoother.engage_exit_ratio = 0.5;


velocity_smoother.stop_dist_to_prohibit_engage = 0.5;
% Minimum stop distance for engagement [m]



%% ============================================================
% STOPPING PARAMETERS
% =============================================================

velocity_smoother.stopping_velocity = 2.778;
% Velocity before stopping point [m/s]


velocity_smoother.stopping_distance = 0.0;
% Distance where stopping velocity is applied [m]



%% ============================================================
% TRAJECTORY EXTRACTION PARAMETERS
% =============================================================

velocity_smoother.extract_ahead_dist = 200.0;
% Forward trajectory extraction distance [m]


velocity_smoother.extract_behind_dist = 5.0;
% Backward trajectory extraction distance [m]


velocity_smoother.delta_yaw_threshold = 1.0472;
% Allowed yaw difference between ego and trajectory [rad]



%% ============================================================
% SIMPLIFIED VALUES FOR 4WS IMPLEMENTATION
%
% Derived quantities used in Simulink
% =============================================================


velocity_smoother.curvature_interval = max( ...
    round(velocity_smoother.curvature_calculation_distance / ...
    velocity_smoother.input_points_interval), ...
    1);

% Equivalent to Autoware:
%
% curvature_calculation_interval =
% max(static_cast<int>(
% curvature_calculation_distance / points_interval),1)
%
% smoother_base.cpp


%% ============================================================
% CREATE VECTOR FOR MASK / FROM WORKSPACE IF REQUIRED
% =============================================================

velocity_smoother_row = [velocity_smoother.curvature_calculation_distance, ...
    velocity_smoother.curvature_threshold,...
    velocity_smoother.resample_ds,...
    velocity_smoother.input_points_interval,...
    velocity_smoother.min_curve_velocity,...
    ];

% wrap into From Workspace numeric format
vel_smoother.time = 0;
vel_smoother.signals.values = velocity_smoother_row;     % 1xN numeric row
vel_smoother.signals.dimensions = length(velocity_smoother_row);

