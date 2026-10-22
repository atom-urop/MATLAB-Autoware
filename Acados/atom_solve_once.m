function out = atom_solve_once(solver, cfg, data)
% Update one OCP and solve it.
%
% This function returns a solution.
% It does not send commands to the vehicle.

N = cfg.N;

%% Validate dimensions
xInit = data.x_init(:);

assert(numel(xInit) == cfg.nx);
assert(all(isfinite(xInit)));

assert(isequal(size(data.reference), [cfg.nye, N+1]));
assert(all(isfinite(data.reference(:))));

assert(numel(data.a_des) == N+1);

assert(isequal(size(data.A), [cfg.nFaces, 2, N+1]));
assert(isequal(size(data.b), [cfg.nFaces, N+1]));
assert(isequal(size(data.epsilon), [4, N]));

%% Keep heading references on a continuous angular branch
reference = data.reference;

psiReference = unwrap(reference(3, :));

branchShift = 2*pi * round( ...
    (xInit(3) - psiReference(1)) / (2*pi));

reference(3, :) = psiReference + branchShift;

%% Update the fixed initial state
solver.set('constr_x0', xInit);

%% Update parameters and references at every node
for k = 0:N

    column = k + 1;

    if k < N
        cornerAllowance = data.epsilon(:, column);
    else
        % Terminal constraint has no following-interval allowance.
        cornerAllowance = zeros(4, 1);
    end

    pk = atom_pack_stage( ...
        cfg, ...
        data.a_des(column), ...
        data.A(:, :, column), ...
        data.b(:, column), ...
        cornerAllowance);

    solver.set('p', pk, k);

    if k < N
        % Running reference:
        % [Xref; Yref; psiref; deltaFref; deltaRref; 0; 0]
        yReference = [reference(:, column); zeros(2, 1)];
    else
        % Terminal reference has no input components.
        yReference = reference(:, column);
    end

    solver.set('cost_y_ref', yReference, k);

end

%% Initial guesses
if isfield(data, 'x_guess')
    xGuess = data.x_guess;
else
    xGuess = repmat(xInit, 1, N+1);
end

if isfield(data, 'u_guess')
    uGuess = data.u_guess;
else
    uGuess = zeros(cfg.nu, N);
end

assert(isequal(size(xGuess), [cfg.nx, N+1]));
assert(isequal(size(uGuess), [cfg.nu, N]));

assert(all(isfinite(xGuess(:))));
assert(all(isfinite(uGuess(:))));

xGuess(:, 1) = xInit;

solver.set('init_x', xGuess);
solver.set('init_u', uGuess);
solver.set('init_pi', zeros(cfg.nx, N));

%% Solve
solver.solve();

%% Retrieve results
out.status       = solver.get('status');
out.X            = solver.get('x');
out.U            = solver.get('u');
out.solveSeconds = solver.get('time_tot');

out.u0 = nan(cfg.nu, 1);

if out.status == 0 ...
        && all(isfinite(out.X(:))) ...
        && all(isfinite(out.U(:)))

    out.u0 = out.U(:, 1);

end

end