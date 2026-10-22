function [model, check] = atom_model(cfg)
% Symbolic dynamics and nonlinear collision constraints.

import casadi.*

%% Symbolic variables
x    = SX.sym('x',    cfg.nx, 1);
xdot = SX.sym('xdot', cfg.nx, 1);
u    = SX.sym('u',    cfg.nu, 1);
p    = SX.sym('p',    cfg.np, 1);

%% State names
X      = x(1);
Y      = x(2);
psi    = x(3);
vx     = x(4);
deltaF = x(5);
deltaR = x(6);
accel  = x(7);
cmdF   = x(8);
cmdR   = x(9);

%% Inputs: rates of commanded steering angles
omegaF = u(1);
omegaR = u(2);

%% Unpack parameters
M = cfg.nFaces;

aDes = p(1);

A = reshape(p(2 : 1 + 2*M), M, 2);

b = p(2 + 2*M : 1 + 3*M);

epsilon = p(2 + 3*M : 5 + 3*M);

%% Kinematic quantities
v = vx / cos(deltaR);

yawRate = (vx / cfg.L) * ...
    (tan(deltaF) - tan(deltaR));

%% Continuous-time state derivatives
f = vertcat( ...
    v * cos(psi + deltaR), ...
    v * sin(psi + deltaR), ...
    yawRate, ...
    accel + yawRate * vx * tan(deltaR), ...
    (cmdF - deltaF) / cfg.tauSteer, ...
    (cmdR - deltaR) / cfg.tauSteer, ...
    (aDes - accel) / cfg.tauAccel, ...
    omegaF, ...
    omegaR ...
);

%% Expanded rectangle in world coordinates
rotation = [ ...
    cos(psi), -sin(psi);
    sin(psi),  cos(psi)
];

position = [X; Y];

hInterval = SX.zeros(cfg.nh, 1);
hTerminal = SX.zeros(cfg.nh, 1);

for j = 1:4

    cornerWorld = position + rotation * cfg.rBody(:, j);

    rows = (j - 1)*M + (1:M);

    % Ordinary footprint containment.
    hTerminal(rows) = A * cornerWorld - b;

    % Containment with room reserved for interval motion.
    % A is normalized by atom_pack_stage().
    hInterval(rows) = ...
        A * cornerWorld - b + epsilon(j) * ones(M, 1);

end

%% Acados model
model = AcadosModel();

model.name = cfg.modelName;

model.x    = x;
model.xdot = xdot;
model.u    = u;
model.p    = p;

model.f_expl_expr = f;
model.f_impl_expr = xdot - f;

% Initial interval: k = 0.
model.con_h_expr_0 = hInterval;

% Intermediate intervals: k = 1,...,N-1.
model.con_h_expr = hInterval;

% Terminal node: k = N.
model.con_h_expr_e = hTerminal;

%% Numerical evaluation helpers for checking the formulation
check.f = Function( ...
    'atom_check_f', {x, u, p}, {f});

check.hInterval = Function( ...
    'atom_check_h_interval', {x, p}, {hInterval});

check.hTerminal = Function( ...
    'atom_check_h_terminal', {x, p}, {hTerminal});

end