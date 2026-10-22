function p = atom_pack_stage(cfg, aDes, A, b, epsilon)
% Assemble one stage's parameter vector.
%
% A       : 4 x 2 corridor boundary normals
% b       : 4 x 1 corridor boundary offsets
% epsilon : 4 x 1 corner movement allowances, in metres
%
% Corridor definition: A*q <= b.

M = cfg.nFaces;

b       = b(:);
epsilon = epsilon(:);

assert(isscalar(aDes), 'aDes must be scalar.');
assert(isequal(size(A), [M, 2]), 'Incorrect A dimensions.');
assert(numel(b) == M, 'Incorrect b dimensions.');
assert(numel(epsilon) == 4, 'Provide four corner allowances.');

values = [aDes; A(:); b; epsilon];

assert(isreal(values) && all(isfinite(values)), ...
    'Stage parameters must be finite real numbers.');

assert(all(epsilon >= 0), ...
    'Movement allowances must be nonnegative.');

%% Normalize each corridor boundary
normalLength = sqrt(sum(A.^2, 2));

assert(all(normalLength > 1e-12), ...
    'Every corridor boundary must have a nonzero normal.');

A = bsxfun(@rdivide, A, normalLength);
b = b ./ normalLength;

%% MATLAB and CasADi both use column-major reshape here
p = [aDes; A(:); b; epsilon];

assert(numel(p) == cfg.np, 'Incorrect parameter-vector length.');

end