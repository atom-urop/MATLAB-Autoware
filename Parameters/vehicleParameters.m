vehicle.param.wheel_radius = 0.313; % The radius of the wheel, primarily used for dead reckoning.
vehicle.param.wheel_width = 0.235; % The lateral width of a wheel tire, primarily used for dead reckoning.
vehicle.param.wheel_base =  2.0; % between front wheel center and rear wheel center
vehicle.param.wheel_tread = 1.9; % between left wheel center and right wheel center
vehicle.param.front_overhang = 0.46; % between front wheel center and vehicle front
vehicle.param.rear_overhang = 0.46; % between rear wheel center and vehicle rear
vehicle.param.left_overhang = 0.128; % between left wheel center and vehicle left
vehicle.param.right_overhang = 0.128; % between right wheel center and vehicle right
vehicle.param.vehicle_height = 1.8;
vehicle.param.max_steer_angle = 0.70; % [rad]


% Parameter for rear_steering_ratio is created for 4WS implementation:

vehicle.param.rear_steering_ratio = -0.2;

vehicle.sim.vehicle_model_type = "DELAY_STEER_ACC_GEARED"; % options: IDEAL_STEER_VEL / IDEAL_STEER_ACC / IDEAL_STEER_ACC_GEARED / DELAY_STEER_ACC / DELAY_STEER_ACC_GEARED
vehicle.sim.initialize_source = "INITIAL_POSE_TOPIC"; %  options: ORIGIN / INITIAL_POSE_TOPIC
vehicle.sim.timer_sampling_time_ms = 25;
vehicle.sim.add_measurement_noise = false; % the Gaussian noise is added to the simulated results
vehicle.sim.vel_lim = 50.0; % limit of velocity
vehicle.sim.vel_rate_lim = 7.0; % limit of acceleration
vehicle.sim.steer_lim = 1.0; % limit of steering angle
vehicle.sim.steer_rate_lim = 5.0; % limit of steering angle change rate
vehicle.sim.acc_time_delay = 0.1; % dead time for the acceleration input
vehicle.sim.acc_time_constant = 0.1; % time constant of the 1st-order acceleration dynamics
vehicle.sim.steer_time_delay = 0.24; % dead time for the steering input
vehicle.sim.steer_time_constant = 0.27; % time constant of the 1st-order steering dynamics
vehicle.sim.x_stddev = 0.0001; % x standard deviation for dummy covariance in map coordinate
vehicle.sim.y_stddev = 0.0001; % y standard deviation for dummy covariance in map coordinate

% Default values form declare_parameter in initialize_vehicle_model in 
% simple_planning_simulator_core.cpp, not declared in
% simulator_model.param.yaml therefore.
vehicle.sim.steer_dead_band = 0.0;
vehicle.sim.steer_bias = 0.0;
vehicle.sim.debug_acc_scaling_factor = 1.0;
vehicle.sim.debug_steer_scaling_factor = 1.0;


% create numeric vector with fields in a fixed order
% order: [wheel_radius, wheel_width, wheel_base, wheel_tread, front_overhang, rear_overhang, left_overhang, right_overhang, vehicle_height, max_steer_angle, vehicle_model_type_code, initialize_source_code, timer_sampling_time_ms, add_measurement_noise_flag, vel_lim, vel_rate_lim, steer_lim, steer_rate_lim, acc_time_delay, acc_time_constant, steer_time_delay, steer_time_constant, x_stddev, y_stddev]
vehicle_model_type_code = 1;      % choose numeric codes, e.g. 1=DELAY_STEER_ACC_GEARED
initialize_source_code   = 2;     % e.g. 1=ORIGIN, 2=INITIAL_POSE_TOPIC
add_noise_flag = double(vehicle.sim.add_measurement_noise); % 0 or 1



veh_row = [ ...
    vehicle.param.wheel_radius, vehicle.param.wheel_width, vehicle.param.wheel_base, vehicle.param.wheel_tread, ...
    vehicle.param.front_overhang, vehicle.param.rear_overhang, vehicle.param.left_overhang, vehicle.param.right_overhang, ...
    vehicle.param.vehicle_height, vehicle.param.max_steer_angle, vehicle_model_type_code, initialize_source_code, ...
    vehicle.sim.timer_sampling_time_ms, add_noise_flag, vehicle.sim.vel_lim, vehicle.sim.vel_rate_lim, ...
    vehicle.sim.steer_lim, vehicle.sim.steer_rate_lim, vehicle.sim.acc_time_delay, vehicle.sim.acc_time_constant, ...
    vehicle.sim.steer_time_delay, vehicle.sim.steer_time_constant, vehicle.sim.x_stddev, vehicle.sim.y_stddev, ...
    vehicle.sim.steer_dead_band, vehicle.sim.steer_bias, vehicle.sim.debug_acc_scaling_factor, vehicle.sim.debug_steer_scaling_factor, ...
    vehicle.param.rear_steering_ratio];

% wrap into From Workspace numeric format
veh.time = 0;
veh.signals.values = veh_row;     % 1xN numeric row
veh.signals.dimensions = length(veh_row);
