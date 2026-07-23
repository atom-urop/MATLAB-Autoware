%% ============================================================
% STANLEY PARAMETERS
%
% We need to create a new file called stanley.param.yaml in /mnt/ssd/autoware/src/launcher/autoware_launch/autoware_launch/
% config/control/trajectory_follower/lateral/.
%
%% ============================================================

stanley.traj_resample_dist = 0.1;    % path resampling interval [m]

% steer offset
stanley.enable_auto_steering_offset_removal = true;
stanley.update_vel_threshold = 5.56;
stanley.update_steer_threshold = 0.035;
stanley.average_num = 1000;
stanley.steering_offset_limit = 0.02;


% Coefficients for Stanley equation

stanley.k_gain = 1;
stanley.k_soft = 2;
stanley.gain_4WS = 0.8;