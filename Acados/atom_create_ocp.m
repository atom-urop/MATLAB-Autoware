function [ocp, check] = atom_create_ocp(cfg)
% Assemble dynamics, cost, constraints and solver options.

%% Basic configuration checks
assert(cfg.N >= 1 && cfg.N == round(cfg.N));
assert(cfg.Tprediction > 0);
assert(cfg.Tcontroller > 0);
assert(cfg.tauSteer > 0 && cfg.tauAccel > 0);

assert(numel(cfg.deltaMax) == 2);
assert(all(cfg.deltaMax > 0 & cfg.deltaMax < pi/2));

assert(numel(cfg.omegaMax) == 2);
assert(all(cfg.omegaMax > 0));

assert(numel(cfg.wRunning) == cfg.ny);
assert(numel(cfg.wTerminal) == cfg.nye);

assert(all(isfinite(cfg.wRunning)));
assert(all(isfinite(cfg.wTerminal)));
assert(all(cfg.wRunning >= 0));
assert(all(cfg.wTerminal >= 0));

% Positive input-rate weights.
assert(all(cfg.wRunning(6:7) > 0));

%% Model
[model, check] = atom_model(cfg);

ocp = AcadosOcp();

ocp.name  = cfg.modelName;
ocp.model = model;

%% Running-cost output matrices
% y = Vx*x + Vu*u
%
% x = [X,Y,psi,vx,deltaF,deltaR,a,cmdF,cmdR]'
% u = [omegaF,omegaR]'

Vx = zeros(cfg.ny, cfg.nx);
Vu = zeros(cfg.ny, cfg.nu);

selectedStates = [1, 2, 3, 5, 6];

Vx(1:5, selectedStates) = eye(5);
Vu(6:7, :) = eye(2);

W  = diag(cfg.wRunning(:));
We = diag(cfg.wTerminal(:));

%% Initial running cost
ocp.cost.cost_type_0 = 'LINEAR_LS';

ocp.cost.Vx_0   = Vx;
ocp.cost.Vu_0   = Vu;
ocp.cost.W_0    = W;
ocp.cost.yref_0 = zeros(cfg.ny, 1);

%% Intermediate running cost
ocp.cost.cost_type = 'LINEAR_LS';

ocp.cost.Vx   = Vx;
ocp.cost.Vu   = Vu;
ocp.cost.W    = W;
ocp.cost.yref = zeros(cfg.ny, 1);

%% Terminal cost
ocp.cost.cost_type_e = 'LINEAR_LS';

ocp.cost.Vx_e   = Vx(1:5, :);
ocp.cost.W_e    = We;
ocp.cost.yref_e = zeros(cfg.nye, 1);

%% Initial state equality
% This convenience property creates:
% idxbx_0 = 0:8
% lbx_0 = ubx_0 = x0
%
% The zeros are construction values.
% Replace them with x_init before every real solve.
ocp.constraints.x0 = zeros(cfg.nx, 1);

%% Input bounds
% Acados index arrays are ZERO-BASED, including in MATLAB.
ocp.constraints.idxbu = [0; 1];

ocp.constraints.lbu = -cfg.omegaMax(:);
ocp.constraints.ubu =  cfg.omegaMax(:);

%% State bounds
% Actual steering angles and commanded steering angles:
% MATLAB positions [5,6,8,9] -> acados indices [4,5,7,8].

idxbx = [4; 5; 7; 8];

steeringLimits = [cfg.deltaMax(:); cfg.deltaMax(:)];

lbx = -steeringLimits;
ubx =  steeringLimits;

% Optional vx bound.
if ~isempty(cfg.vxBounds)

    assert(numel(cfg.vxBounds) == 2);
    assert(all(isfinite(cfg.vxBounds)));
    assert(cfg.vxBounds(1) <= cfg.vxBounds(2));

    idxbx = [3; idxbx];

    lbx = [cfg.vxBounds(1); lbx];
    ubx = [cfg.vxBounds(2); ubx];

end

% Intermediate nodes.
ocp.constraints.idxbx = idxbx;
ocp.constraints.lbx   = lbx;
ocp.constraints.ubx   = ubx;

% Terminal node.
ocp.constraints.idxbx_e = idxbx;
ocp.constraints.lbx_e   = lbx;
ocp.constraints.ubx_e   = ubx;

%% Nonlinear collision constraints
% Mathematical bounds: -infinity <= h <= 0.
%
% Use acados' infinity sentinel with HPIPM.
minusInfinity = -get_acados_infty() * ones(cfg.nh, 1);
zeroUpper     = zeros(cfg.nh, 1);

ocp.constraints.lh_0 = minusInfinity;
ocp.constraints.uh_0 = zeroUpper;

ocp.constraints.lh = minusInfinity;
ocp.constraints.uh = zeroUpper;

ocp.constraints.lh_e = minusInfinity;
ocp.constraints.uh_e = zeroUpper;

%% General linear constraints: none (ng = 0)

%% Soft collision constraints (slack variables)
% With hard corridor constraints the QP becomes infeasible (status 4) as
% soon as a car corner cannot be kept inside its corridor, e.g. when the
% rate-limited steering lags a sharp planned steering change near obstacles.
% A slack s >= 0 per row turns h <= 0 into h <= s, penalised with
% 0.5*softWeight*s^2: the QP is always feasible and s = 0 whenever possible.
idxsh  = (0:cfg.nh-1)';                % all 16 collision rows, zero-based
Zslack = cfg.softWeight * ones(cfg.nh, 1);
zslack = zeros(cfg.nh, 1);             % no linear term

ocp.constraints.idxsh_0 = idxsh;
ocp.constraints.idxsh   = idxsh;
ocp.constraints.idxsh_e = idxsh;

ocp.cost.Zl_0 = Zslack;  ocp.cost.Zu_0 = Zslack;  ocp.cost.zl_0 = zslack;  ocp.cost.zu_0 = zslack;
ocp.cost.Zl   = Zslack;  ocp.cost.Zu   = Zslack;  ocp.cost.zl   = zslack;  ocp.cost.zu   = zslack;
ocp.cost.Zl_e = Zslack;  ocp.cost.Zu_e = Zslack;  ocp.cost.zl_e = zslack;  ocp.cost.zu_e = zslack;

%% Construction parameters
% Synthetic corridor for building and stationary testing only.
% It is NOT a corridor extracted from the actual map.
Aconstruction = [ ...
     1,  0;
    -1,  0;
     0,  1;
     0, -1
];

bConstruction = [5; 5; 4; 4];

ocp.parameter_values = atom_pack_stage( ...
    cfg, 0, Aconstruction, bConstruction, zeros(4, 1));

%% Prediction grid
ocp.solver_options.N_horizon = cfg.N;
ocp.solver_options.tf        = cfg.horizon;

ocp.solver_options.time_steps = ...
    cfg.Tprediction * ones(cfg.N, 1);

%% Cost discretization
% J = sum_k Tprediction * ell_k + m_N.
%
% Keep W and We unscaled.
% Apply the time-step scaling once, here.
ocp.solver_options.cost_discretization = 'EULER';

ocp.solver_options.cost_scaling = [ ...
    cfg.Tprediction * ones(cfg.N, 1);
    1
];

%% Dynamics integration
ocp.solver_options.integrator_type = 'ERK';

ocp.solver_options.sim_method_num_stages = 4;
ocp.solver_options.sim_method_num_steps  = ...
    cfg.integrationSubsteps;

%% Nonlinear and quadratic-program solvers
ocp.solver_options.nlp_solver_type = cfg.nlpSolver;

ocp.solver_options.hessian_approx = 'GAUSS_NEWTON';
ocp.solver_options.globalization =  'FIXED_STEP';
% ocp.solver_options.globalization_fixed_step_length = 0.7;   % T1 test: rejected (no fix, slower)
% ocp.solver_options.globalization_use_SOC = 1;               % T2 test: second-order correction

ocp.solver_options.qp_solver = ...
    'PARTIAL_CONDENSING_HPIPM';

ocp.solver_options.qp_solver_cond_N = min(5, cfg.N);

% Use Riccati variants that accommodate semidefinite state costs.
ocp.solver_options.qp_solver_ric_alg      = 0;
ocp.solver_options.qp_solver_cond_ric_alg = 0;

% Small numerical regularization.
ocp.solver_options.levenberg_marquardt = 1e-6;

ocp.solver_options.nlp_solver_max_iter = cfg.maxIter;
    
ocp.solver_options.nlp_solver_tol_stat = cfg.nlpTolStat;
ocp.solver_options.nlp_solver_tol_eq   = cfg.nlpTol;
ocp.solver_options.nlp_solver_tol_ineq = cfg.nlpTol;
ocp.solver_options.nlp_solver_tol_comp = cfg.nlpTol;

ocp.solver_options.print_level = 0;

%% Code generation
% These locations are used only when YOU construct the solver.
ocp.code_gen_options.code_export_directory = ...
    fullfile(pwd, 'c_generated_atom');

ocp.code_gen_options.json_file = 'atom_ocp.json';

%% Simulink S-function (Stage II)
% Only the ports the parking controller needs. Sample time inherited ('-1'):
% the block runs at the 0.04 s rate of parkingNMPC, not at the 0.12 s node
% spacing. No extra Simulink model file is generated.
sopts = AcadosOcpSimulinkOptions();
sopts.samplingtime = '-1';
sopts.generate_simulink_block = 0;

inNames = fieldnames(sopts.inputs.to_struct());
for i = 1:numel(inNames)
    sopts.inputs.(inNames{i}) = 0;      % every input off ...
end
sopts.inputs.lbx_0          = 1;        % ... except these
sopts.inputs.ubx_0          = 1;
sopts.inputs.parameter_traj = 1;
sopts.inputs.y_ref_0        = 1;
sopts.inputs.y_ref          = 1;
sopts.inputs.y_ref_e        = 1;

outNames = fieldnames(sopts.outputs.to_struct());
for i = 1:numel(outNames)
    sopts.outputs.(outNames{i}) = 0;    % every output off ...
end
sopts.outputs.u0            = 1;        % ... except these
sopts.outputs.solver_status = 1;
sopts.outputs.sqp_iter      = 1;
sopts.outputs.CPU_time      = 1;

sopts.inputs.x_init = 1; sopts.inputs.u_init = 1; sopts.inputs.pi_init = 1;   % same initial guess as Stage I
ocp.simulink_opts = sopts;

end