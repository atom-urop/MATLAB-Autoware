function cfg = atom_config(vehicle)
% ATOM NMPC configuration.
%
% vehicle is the structure created by vehicleParameters.m.

assert(isstruct(vehicle) ...
    && isfield(vehicle, 'param') ...
    && isfield(vehicle, 'sim'), ...
    'Pass the vehicle structure created by vehicleParameters.m.');

vp = vehicle.param;
vs = vehicle.sim;

%% Model identity
cfg.modelName = 'atom_4ws_nmpc';

%% Controller timing: design choices
cfg.N           = 15;
cfg.Tprediction = 0.12;
cfg.Tcontroller = 0.04;

%% Physical steering transport delay
cfg.Tdelay = vs.steer_time_delay;

cfg.horizon = cfg.N * cfg.Tprediction;

cfg.nodeTimeOffsets = ...
    cfg.Tdelay + (0:cfg.N) * cfg.Tprediction;

%% Vehicle and actuator parameters
cfg.L        = vp.wheel_base;
cfg.tauSteer = vs.steer_time_constant;
cfg.tauAccel = vs.acc_time_constant;

%% Steering-angle limits
% Existing parameter source provides a shared limit.
% Order: [front; rear]
cfg.deltaMax = repmat(vp.max_steer_angle, 2, 1);

%% Commanded steering-rate limits
% Preserve the existing controller's choice to use this value.

%% Commanded steering-rate limits (controller design choice)
% vehicle.sim.steer_rate_lim (5 rad/s) = simulator limit on the ACTUAL wheel rate.
% omegaMax = limit on the rate of the COMMANDED angle (NMPC input), chosen lower for now.
cfg.omegaMax = [1.0; 1.0];                 % [front; rear] rad/s
assert(all(cfg.omegaMax <= vs.steer_rate_lim), ...
    'Commanded rate limit above the actuator rate limit.');

%% Optional longitudinal-speed constraint
% Remains undecided in the latest handwritten formulation.
cfg.vxBounds = [];

%% Collision clearance: controller setting
% Same convention as the existing parking controller:
% 0.5 m total expansion means 0.25 m on each face.
cfg.collisionMargin = 0.5;

%% Soft collision constraints
% Quadratic penalty on how far a car corner is outside its corridor.
% Keeps every QP feasible; violations stay at the millimetre level
% whenever the corridor can be respected.
cfg.softWeight = 1e5;

marginPerFace = cfg.collisionMargin / 2;

%% Expanded vehicle footprint
cfg.F = ...
    vp.wheel_base + vp.front_overhang + marginPerFace;

cfg.B = ...
    vp.rear_overhang + marginPerFace;

cfg.halfWidth = ...
    (vp.wheel_tread + vp.left_overhang + vp.right_overhang)/2 ...
    + marginPerFace;

cfg.rBody = [ ...
     cfg.F,          cfg.F,         -cfg.B,         -cfg.B;
     cfg.halfWidth, -cfg.halfWidth,  -cfg.halfWidth,  cfg.halfWidth
];

%% Formulation dimensions
cfg.nx  = 9;
cfg.nu  = 2;
cfg.ny  = 7;
cfg.nye = 5;

cfg.nFaces = 4;
cfg.nh     = 4 * cfg.nFaces;

% p = [a_des; A(:); b; epsilon]
cfg.np = 1 + 2*cfg.nFaces + cfg.nFaces + 4;

%% Cost weights: equivalent to parkingNMPC w = [10 10 1 0.1 0.1]
% fmincon sums w*e^2 every Ts = 0.04 s; acados integrates 1/2*||y-yref||_W^2
% with EULER scaling -> W = 2*w/Ts, terminal W_e = 2*w.
% Rate term: fmincon penalises 0.1*(Ts*omega)^2 -> 0.1*Ts^2 on omega.
Ts     = cfg.Tcontroller;
wState = [10; 10; 1; 0.1; 0.1];          % X, Y, psi, deltaF, deltaR
wRate  = 0.1*Ts^2*[1; 1];                % omegaF, omegaR
cfg.wRunning  = 2*[wState; wRate]/Ts;    % diag(500,500,50,5,5,0.008,0.008)
cfg.wTerminal = 2*wState;                % diag(20,20,2,0.2,0.2)

%% Numerical solver settings
cfg.nlpSolver = 'SQP';
cfg.nlpTol   = 1e-6;
cfg.nlpTolStat = 1e-4;   % stationarity only
cfg.maxIter  = 50;

cfg.integrationSubsteps = 3;

end