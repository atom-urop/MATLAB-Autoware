function [solver, cfg, ocp] = generate_atom_solver(vehicle)
% Generate the ATOM solver and run a stationary consistency check.
%
% Running this function creates generated files on YOUR machine.

%% Assemble the formulation
cfg = atom_config(vehicle);

[ocp, check] = atom_create_ocp(cfg);

%% Synthetic stationary test data
A = [ ...
    1,  0;
    -1,  0;
    0,  1;
    0, -1
    ];

b = [5; 5; 4; 4];

xStationary = zeros(cfg.nx, 1);
uStationary = zeros(cfg.nu, 1);

pStationary = atom_pack_stage( ...
    cfg, 0, A, b, zeros(4, 1));

%% Check the symbolic equations before compilation
fStationary = full(check.f( ...
    xStationary, uStationary, pStationary));

hStationary = full(check.hInterval( ...
    xStationary, pStationary));

assert(norm(fStationary, inf) < 1e-12, ...
    'Stationary state should have zero derivative.');

assert(all(hStationary <= 0), ...
    'Stationary footprint must fit inside the test corridor.');

%% Generate and compile the solver
solver = AcadosOcpSolver(ocp);

%% Compile the Simulink S-function (Stage II)
here = pwd;
cd(ocp.code_gen_options.code_export_directory);
make_sfun;                  % -> acados_solver_sfunction_atom_4ws_nmpc.mexa64
cd(here);

addpath("/home/husain5/MATLAB-Autoware/Acados/c_generated_atom");   % S-function location

%% Prepare one stationary OCP
data.x_init = xStationary;

data.reference = zeros(cfg.nye, cfg.N+1);
data.a_des     = zeros(1, cfg.N+1);

data.A = repmat(A, 1, 1, cfg.N+1);
data.b = repmat(b, 1, cfg.N+1);

% With vx = a = a_des = 0, this model has no body motion.
% Zero movement allowance is appropriate for this test.
data.epsilon = zeros(4, cfg.N);

data.x_guess = zeros(cfg.nx, cfg.N+1);
data.u_guess = zeros(cfg.nu, cfg.N);

%% Solve and inspect
out = atom_solve_once(solver, cfg, data);

assert(out.status == 0, ...
    sprintf('Stationary solve failed: status %d.', out.status));

assert(all(isfinite(out.X(:))));
assert(all(isfinite(out.U(:))));

assert(max(abs(out.X(:))) < 1e-5, ...
    'Unexpected motion in the stationary test.');

assert(max(abs(out.U(:))) < 1e-5, ...
    'Unexpected steering input in the stationary test.');

fprintf('ATOM solver generated.\n');
fprintf('Stationary test passed.\n');
fprintf('States: %d, inputs: %d, parameters: %d.\n', ...
    cfg.nx, cfg.nu, cfg.np);
fprintf('Prediction horizon: %.2f s.\n', cfg.horizon);
fprintf('Solver time for this test: %.3f ms.\n', ...
    1000*out.solveSeconds);

end