function d_state = vehicleDynamics(state, command, vehicle)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This vehicleDynamics model follows DELAY_STEER_ACC_GEARED
% model, but it doesn't take care about delay modeling of acceleration and steer, 
% neither presents a model for gear management
% since they're not something we're going to modify on Autoware.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Current state

x = state.x;
y = state.y;
yaw = state.yaw;

vel = state.vx;
steer = state.steer;
acc = state.acc;


%% Input commands

acc_des = command.Longitudinal.acceleration;

steer_des = command.Lateral.front_steering_tire_angle;


%% Parameters

wheelbase = vehicle.param.wheel_base;

acc_tau = vehicle.sim.acc_time_constant;


%% State derivative

d_state.x = vel*cos(yaw);

d_state.y = vel*sin(yaw);

d_state.yaw = vel*tan(steer)/wheelbase;

d_state.vx = acc;

d_state.steer = steer_des;

d_state.acc = -(acc - acc_des)/acc_tau;


end