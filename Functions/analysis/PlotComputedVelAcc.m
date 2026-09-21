function PlotComputedVelAcc(logData, kIdx)
%PLOTCOMPUTEDVELACC  Scatter the planned (computed) velocity / acceleration
%   profile of ONE trajectory snapshot produced by the velocity smoother.
%
%   PlotComputedVelAcc(out.log)      plots the last logged trajectory
%   PlotComputedVelAcc(out.log, k)   plots the trajectory logged at step k
%
%   Expected signal order into the Mux / Matrix Concatenate block:
%       1 -> longitudinal_velocity_mps
%       2 -> acceleration_mps2
%       3 -> x
%       4 -> y
%
%   Handles the three shapes To Workspace can hand back:
%       [Npoints x 4 x Nsteps]  (matrix signal, e.g. Matrix Concatenate dim 2)
%       [Npoints x 4]           (single snapshot)
%       [Nsteps  x 4*Npoints]   (Mux flattens the four vectors into one column)

    if nargin < 2, kIdx = []; end

    % ---- unwrap the To Workspace save format --------------------------
    if isa(logData, 'timeseries')
        logData = logData.Data;
    elseif isstruct(logData) && isfield(logData, 'signals')
        logData = logData.signals.values;
    end
    logData = double(logData);

    % ---- bring the data to a single [Npoints x 4] snapshot -------------
    sz = size(logData);
    if ndims(logData) == 3                          % [Np x 4 x Nsteps]
        if isempty(kIdx), kIdx = sz(3); end
        M = logData(:, :, kIdx);
    elseif sz(2) == 4 && sz(1) > 1                  % [Np x 4]
        kIdx = 1;
        M = logData;
    elseif mod(sz(2), 4) == 0                       % [Nsteps x 4*Np], Mux order
        if isempty(kIdx), kIdx = sz(1); end
        M = reshape(logData(kIdx, :), sz(2)/4, 4);  % col-major: [v a x y]
    else
        error('PlotComputedVelAcc:badShape', ...
              'Unexpected log size %s - check the To Workspace block.', mat2str(sz));
    end

    v = M(:,1);  a = M(:,2);  x = M(:,3);  y = M(:,4);

    % ---- drop the zero padding of the fixed-size trajectory array ------
    lastValid = find(~(x == 0 & y == 0 & v == 0), 1, 'last');
    if isempty(lastValid)
        error('PlotComputedVelAcc:allZeros', ...
              ['Snapshot %d contains only zeros. Check that you selected ' ...
               'longitudinal_velocity_mps (not lateral) and that the smoother ' ...
               'had a valid trajectory at this step.'], kIdx);
    end
    keep = 1:lastValid;
    x = x(keep);  y = y(keep);  v = v(keep);  a = a(keep);

    fprintf(['snapshot %d: %d points | v in [%.2f, %.2f] m/s | ' ...
             'a in [%.2f, %.2f] m/s^2\n'], ...
            kIdx, numel(x), min(v), max(v), min(a), max(a));

    figure;
    scatter(x, y, 20, v, 'filled');
    colormap turbo; c = colorbar; c.Label.String = 'velocity [m/s]';
    axis equal; grid on; xlabel('x [m]'); ylabel('y [m]');
    title(sprintf('Computed velocity along path (step %d)', kIdx));

    figure;
    scatter(x, y, 20, a, 'filled');
    colormap turbo; c = colorbar; c.Label.String = 'acceleration [m/s^2]';
    axis equal; grid on; xlabel('x [m]'); ylabel('y [m]');
    title(sprintf('Computed acceleration along path (step %d)', kIdx));
end