function x1 = vehicleStep(x0, u, Ts, p)
%VEHICLESTEP  Where is the car one moment later?
%
%   x = [X; Y; psi; vx; df; dr]    position, heading, speed, front/rear angle
%   u = [a_cmd; df_cmd; dr_cmd]    what we ask for
%   Ts                             how long one moment lasts [s]
%   p                              the car's limits
%% This function here is generated as stage 1 of building the nmpc
X = x0(1); Y = x0(2); psi = x0(3); vx = x0(4); df = x0(5); dr = x0(6);
a = u(1); dfc = u(2); drc = u(3);

% 1-3. the wheels move toward what we asked, slowly, and within limits
ddf = max(-p.rate_lim, min(p.rate_lim, (dfc - df)/p.tau_s));
ddr = max(-p.rate_lim, min(p.rate_lim, (drc - dr)/p.tau_s));
df1 = max(-p.steer_max, min(p.steer_max, df + Ts*ddf));
dr1 = max(-p.steer_max, min(p.steer_max, dr + Ts*ddr));

% 4. the speed changes with the pedal
a   = max(-p.a_max, min(p.a_max, a));
vx1 = max(-p.v_max, min(p.v_max, vx + Ts*a));

% 5. the car follows a curve — same formula the planner uses
dr_mid = 0.5*(dr + dr1);
vx_mid = 0.5*(vx + vx1);
d_rear = vx_mid*Ts / max(cos(dr_mid), 0.1);

[X1, Y1, psi1] = bicycleGetPose(X, Y, psi, df1, dr1, p.L, d_rear);

x1 = [X1; Y1; psi1; vx1; df1; dr1];
end