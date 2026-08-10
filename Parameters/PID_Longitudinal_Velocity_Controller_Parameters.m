%%PID controller is the longitudinal velocity controller. The actual
%%controller used in Autoware is the PI controller because it considers Kd= 0;
%%The formula is:
%%function acc_cmd = fcn(a_ff, v_target, v_actual)
% % acc_cmd = a_ff + kp * (v_target - v_actual)
%%OR:
% error(t)   = v_target(t) − v_actual(t)
% acc_pid(t) = kp·error(t) + ki·∫error(τ)dτ + kd·d(error)/dt
% acc_cmd(t) = acc_ff(t) + acc_pid(t)

%%as we are controlling the acceleration as an output using the difference
%%of the velocity, then adding the feedforward term coming from the velocity
%%smoother 
% real Autoware's PidLongitudinalController does exactly this

Kp = 1.0;%P term reacts to how wrong you are right now. Big error → big correction
Ki = 0.1;%I term reacts to how wrong you've consistently been — it catches steady, persistent bias that P alone can't fully close
% max_out = 1.0;
% min_out = -1.0;