function epsilon = atom_swept_allowance( ...
    cfg, pointSpeedBound, yawRateBound)
% Conservative corner displacement bounds.
%
% pointSpeedBound : N values, m/s
% yawRateBound    : N values, rad/s
%
% Output:
% epsilon        : 4 x N, metres
%
% This calculation assumes the supplied bounds are valid.
% It does not establish their validity.

pointSpeedBound = pointSpeedBound(:).';
yawRateBound    = yawRateBound(:).';

assert(numel(pointSpeedBound) == cfg.N);
assert(numel(yawRateBound) == cfg.N);

assert(all(isfinite(pointSpeedBound)));
assert(all(isfinite(yawRateBound)));

assert(all(pointSpeedBound >= 0));
assert(all(yawRateBound >= 0));

%% Distance from rear axle to each expanded-footprint corner
cornerRadius = sqrt(sum(cfg.rBody.^2, 1)).';

%% Translation contribution plus rotation contribution
epsilon = cfg.Tprediction * ( ...
    ones(4, 1) * pointSpeedBound ...
    + cornerRadius * yawRateBound ...
    );

end