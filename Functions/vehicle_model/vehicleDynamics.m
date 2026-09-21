function d_state = vehicleDynamics(state, command, vehicle)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description:
% This vehicleDynamics model follows DELAY_STEER_ACC_GEARED
% model, but it doesn't take care about delay modeling of acceleration and steer, 
% neither presents a model for gear management
% since they're not something we're going to modify on Autoware.
%
% Inputs:
% - state: VehicleInternalStateBus. It contains the state of the vehicle
%          at time instant t. 
% - command: /control/command/control_cmd topic --> ControlCMDBus.
%          It contains the desired acc and steering commands coming from
%          the control region.
%
%
% Parameters:
% - vehicle: vehicle parameters struct defined in vehicleParameters.m.

% Outputs: 
% - d_state: It contains the derivative of the state of the vehicle at time
%          instant t+1.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% ==========================
% Saturation function
% ===========================

sat = @(val, upper, lower) min(max(val,lower),upper);



%% ==========================
% Current state
% ===========================

vel = sat( ...
    state.vx,...
    vehicle.sim.vel_lim,...
   -vehicle.sim.vel_lim);


acc = sat( ...
    state.acc,...
    vehicle.sim.vel_rate_lim,...
   -vehicle.sim.vel_rate_lim);


yaw = state.yaw;

steer_front = state.steer_front;
steer_rear = state.steer_rear;          % Modified for 4WS  


%% ==========================
% Input commands
% ===========================

acc_des = sat( ...
    command.longitudinal.acceleration,...
    vehicle.sim.vel_rate_lim,...
   -vehicle.sim.vel_rate_lim);


acc_des = acc_des * ...
    vehicle.sim.debug_acc_scaling_factor;



steer_front_des = sat( ...
    command.lateral.front_steering_tire_angle,...
    vehicle.sim.steer_lim,...
   -vehicle.sim.steer_lim);


steer_front_des = steer_front_des * ...
    vehicle.sim.debug_steer_scaling_factor;

steer_rear_des = sat( ...                                 % Modified for 4WS
    command.lateral.rear_steering_tire_angle,...
    vehicle.sim.steer_lim,...
    -vehicle.sim.steer_lim);


steer_rear_des = steer_rear_des * ...                     % Modified for 4WS
    vehicle.sim.debug_steer_scaling_factor;



%% ==========================
% Steering dynamics
% ===========================

% getSteer() in simple_planning_simulator corresponds to state.steer
% From the phisical vehicle front_steer and rear_steer are measured.

steer_diff_front = steer_front - steer_front_des;

steer_diff_rear = steer_rear  - steer_rear_des;             % Modified for 4WS


% dead band

if steer_diff_front > vehicle.sim.steer_dead_band

    steer_diff_with_dead_band_front = ...
        steer_diff_front - vehicle.sim.steer_dead_band;


elseif steer_diff_front < -vehicle.sim.steer_dead_band

    steer_diff_with_dead_band_front = ...
        steer_diff_front + vehicle.sim.steer_dead_band;


else

    steer_diff_with_dead_band_front = 0;

end



if steer_diff_rear > vehicle.sim.steer_dead_band            % Modified for 4WS

    steer_diff_with_dead_band_rear = ...
        steer_diff_rear - vehicle.sim.steer_dead_band;


elseif steer_diff_rear < -vehicle.sim.steer_dead_band

    steer_diff_with_dead_band_rear = ...
        steer_diff_rear + vehicle.sim.steer_dead_band;


else

    steer_diff_with_dead_band_rear = 0;

end



steer_rate_front = sat( ...
    -steer_diff_with_dead_band_front / vehicle.sim.steer_time_constant,...
     vehicle.sim.steer_rate_lim,...
    -vehicle.sim.steer_rate_lim);

steer_rate_rear = sat( ...                                                  % Modified for 4WS
    -steer_diff_with_dead_band_rear / vehicle.sim.steer_time_constant,...
    vehicle.sim.steer_rate_lim,...
    -vehicle.sim.steer_rate_lim);



%% ==========================
% State derivative
% ===========================


d_state.x = vel*cos(yaw);

d_state.y = vel*sin(yaw);


d_state.yaw = ...                                                           % Modified for 4WS
    vel*(tan(steer_front)-tan(steer_rear))/vehicle.param.wheel_base;


d_state.vx = acc;


d_state.steer_front = steer_rate_front;

d_state.steer_rear = steer_rate_rear;                                       % Modified for 4WS

d_state.acc = ...
    -(acc-acc_des)/vehicle.sim.acc_time_constant;


end