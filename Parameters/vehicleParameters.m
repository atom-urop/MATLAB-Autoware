%[text] ## Vehicle info parameters
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
%[text] ## Simulator model parameters
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

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":15.5}
%---
