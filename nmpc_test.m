%% nmpc_test.m
%  Run the Simulink model once first, so plan_traj and vehicle exist.

% ---- the car's numbers --------------------------------------------------
p.L         = vehicle.param.wheel_base;
p.tau_s     = vehicle.sim.steer_time_constant;
p.rate_lim  = vehicle.sim.steer_rate_lim;
p.steer_max = vehicle.param.max_steer_angle;
p.a_max     = vehicle.sim.vel_rate_lim;
p.v_max     = 2.0;
p.Ts        = 0.04;
nd = round(vehicle.sim.steer_time_delay / p.Ts);

% ---- the path, as a distance map ---------------------------------------
ref = buildParkingReference(plan_traj);

% ---- settings -----------------------------------------------------------
v0     = 0.3;      % slow walking speed
N      = 15;       % how far ahead the controller looks
s_stop = 5.0;      % drive only the first 5 metres this time

w.ey = 10;  w.psi = 1;  w.d = 1;  w.du = 0.1;

% ---- the car starts on the path, wheels straight -----------------------
x   = [ref.X(1); ref.Y(1); ref.psi(1); v0; 0; 0];
buf = zeros(nd,2);
U   = zeros(N,2);

lo   = -p.steer_max*ones(N,2);
hi   =  p.steer_max*ones(N,2);
opts = optimoptions('fmincon','Algorithm','sqp','Display','none', ...
    'MaxIterations',30);

% ---- drive --------------------------------------------------------------
s = 0;  k = 0;  rec = [];  t0 = tic;
fprintf('driving...\n');

while s < s_stop && k < 1500
    k = k + 1;

    [s, ey] = nearestOnPath(ref, x(1), x(2), s);

    U = fmincon(@(U) nmpcCost(U,x,s,buf,ref,p,w), U, ...
        [],[],[],[], lo,hi,[], opts);

    buf = [buf; U(1,:)];        % the new command joins the back of the queue
    applied = buf(1,:);         % the oldest one reaches the wheels now
    buf(1,:) = [];

    x = vehicleStep(x, [0; applied(1); applied(2)], p.Ts, p);

    rec(k,:) = [k*p.Ts, x(1), x(2), ey, applied(1), applied(2)];

    U = [U(2:end,:); U(end,:)];   % next time, start from this answer

    if mod(k,50) == 0
        fprintf('  %.1f m\n', s);
    end
end

% ---- results ------------------------------------------------------------
fprintf('\ndistance driven        %.2f m\n', s);
fprintf('steps                  %d\n', k);
fprintf('computing time         %.0f s\n', toc(t0));
fprintf('worst sideways error   %.3f m\n', max(abs(rec(:,4))));
fprintf('average sideways error %.3f m\n', mean(abs(rec(:,4))));

figure
subplot(1,2,1)
plot(ref.X, ref.Y, 'k-', 'LineWidth',1.5); hold on
plot(rec(:,2), rec(:,3), 'g-', 'LineWidth',1.3)
axis equal; grid on; legend('the path','with the controller')
xlabel('x [m]'); ylabel('y [m]'); title('where it went')

subplot(1,2,2)
plot(rec(:,1), rec(:,4), 'LineWidth',1.3); grid on
xlabel('t [s]'); ylabel('sideways error [m]'); title('how far off the path')