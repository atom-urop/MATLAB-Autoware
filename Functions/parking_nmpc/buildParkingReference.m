function ref = buildParkingReference(plan_traj)
%BUILDPARKINGREFERENCE  Turn the plan into a map indexed by distance.
%
% The controller asks "I am s metres along the path - what should I be
% doing?"  This builds the table that answers it.

X = plan_traj(:,1);
Y = plan_traj(:,2);

% heading, recovered from the quaternion the bus carries
psi = unwrap( 2*atan2(plan_traj(:,6), plan_traj(:,7)) );

df = plan_traj(:,12);          % planned front steering
dr = plan_traj(:,13);          % planned rear steering

% the distance markers
s = cumsum([0; hypot(diff(X), diff(Y))]);

% forward (+1) or reverse (-1) on this segment
gear = sign(plan_traj(1,8));
if gear == 0, gear = 1; end

% how fast the steering changes per metre travelled
% (we will need this later to work out a safe speed)
q = zeros(size(s));
q(1:end-1) = max(abs(diff(df)), abs(diff(dr))) ./ max(diff(s), eps);
q(end)     = q(end-1);

ref.s     = s;
ref.X     = X;
ref.Y     = Y;
ref.psi   = psi;
ref.df    = df;
ref.dr    = dr;
ref.q     = q;
ref.gear  = gear;
ref.s_end = s(end);
end