function PlotControllerReferences(logData, kIdx)
%PLOTCONTROLLERREFERENCES Plot references received by the controller.
%
% Logged matrix column order:
%   1: x position [m]
%   2: y position [m]
%   3: front steering reference [rad]
%   4: rear steering reference [rad]
%   5: longitudinal velocity reference [m/s]
%   6: acceleration reference [m/s^2]
%
% Examples:
%   PlotControllerReferences(out.controller_reference_log)
%   PlotControllerReferences(out.controller_reference_log, 125)
%
% If Single simulation output is disabled:
%   PlotControllerReferences(controller_reference_log)

if nargin < 2
    kIdx = [];
end

%% Unwrap common Simulink logging formats

if isa(logData, 'Simulink.SimulationData.Signal')
    logData = logData.Values;
end

if isa(logData, 'timeseries')
    logData = logData.Data;

elseif isstruct(logData) && isfield(logData, 'signals')
    logData = logData.signals.values;
end

logData = double(logData);

%% Extract one trajectory snapshot

if ndims(logData) == 3

    % Expected dimensions:
    % [trajectory points x 6 references x simulation steps]
    if size(logData, 2) ~= 6
        error('PlotControllerReferences:InvalidSignalCount', ...
            ['Expected six logged signals, but the logged array ' ...
             'has size %s.'], mat2str(size(logData)));
    end

    numberOfSnapshots = size(logData, 3);

    if isempty(kIdx)
        % Automatically select the last snapshot containing data.
        snapshotHasData = squeeze(any(any(logData ~= 0, 1), 2));
        kIdx = find(snapshotHasData, 1, 'last');

        if isempty(kIdx)
            error('PlotControllerReferences:EmptyLog', ...
                'The controller reference log contains only zeros.');
        end
    end

    if kIdx < 1 || kIdx > numberOfSnapshots || kIdx ~= floor(kIdx)
        error('PlotControllerReferences:InvalidSnapshot', ...
            'Snapshot index must be an integer from 1 to %d.', ...
            numberOfSnapshots);
    end

    M = logData(:, :, kIdx);

elseif ismatrix(logData) && size(logData, 2) == 6

    % A single [trajectory points x 6] snapshot.
    M = logData;
    kIdx = 1;

else
    error('PlotControllerReferences:InvalidLogShape', ...
        ['Unexpected logged-data size %s. Expected either ' ...
         '[points x 6] or [points x 6 x snapshots].'], ...
         mat2str(size(logData)));
end

%% Separate the reference signals

x = M(:,1);
y = M(:,2);

frontReference = M(:,3);
rearReference  = M(:,4);

velocityReference     = M(:,5);
accelerationReference = M(:,6);

trajectoryPoint = (1:size(M,1)).';

%% Remove nonfinite samples

valid = ...
    isfinite(x)                     & ...
    isfinite(y)                     & ...
    isfinite(frontReference)        & ...
    isfinite(rearReference)         & ...
    isfinite(velocityReference)     & ...
    isfinite(accelerationReference);

x = x(valid);
y = y(valid);

frontReference = frontReference(valid);
rearReference  = rearReference(valid);

velocityReference     = velocityReference(valid);
accelerationReference = accelerationReference(valid);

trajectoryPoint = trajectoryPoint(valid);

if isempty(trajectoryPoint)
    error('PlotControllerReferences:NoValidPoints', ...
        'The selected snapshot does not contain valid trajectory points.');
end

%% Dark-theme colors

figureBackground = [0.025 0.025 0.025];

frontColor       = [0.20 0.55 1.00];
rearColor        = [1.00 0.25 0.25];
velocityColor    = [0.15 0.85 0.40];
accelerationColor = [1.00 0.45 0.12];

zeroLineColor = [0.65 0.65 0.65];

%% Figure 1: references versus trajectory-point number

figure( ...
    'Name', 'Controller references by trajectory point', ...
    'Color', figureBackground, ...
    'Position', [100 80 1100 820]);

tiledlayout(3, 1, ...
    'TileSpacing', 'compact', ...
    'Padding', 'compact');

% Front and rear steering references
ax1 = nexttile;

plot(ax1, trajectoryPoint, frontReference, ...
    '-', ...
    'Color', frontColor, ...
    'LineWidth', 2.0);

hold(ax1, 'on');

plot(ax1, trajectoryPoint, rearReference, ...
    '-', ...
    'Color', rearColor, ...
    'LineWidth', 2.0);

yline(ax1, 0, ':', ...
    'Color', zeroLineColor, ...
    'LineWidth', 1.0, ...
    'HandleVisibility', 'off');

grid(ax1, 'on');

ylabel(ax1, 'Steering [rad]');

title(ax1, sprintf( ...
    'References received by controller - snapshot %d', kIdx));

lgd = legend(ax1, ...
    {'Front reference', 'Rear reference'}, ...
    'Location', 'best');

applyDarkStyle(ax1);
applyDarkLegendStyle(lgd);

% Velocity reference
ax2 = nexttile;

plot(ax2, trajectoryPoint, velocityReference, ...
    '-', ...
    'Color', velocityColor, ...
    'LineWidth', 2.0);

grid(ax2, 'on');

ylabel(ax2, 'Velocity [m/s]');
title(ax2, 'Longitudinal velocity reference');

applyDarkStyle(ax2);

% Acceleration reference
ax3 = nexttile;

plot(ax3, trajectoryPoint, accelerationReference, ...
    '-', ...
    'Color', accelerationColor, ...
    'LineWidth', 2.0);

hold(ax3, 'on');

yline(ax3, 0, ':', ...
    'Color', zeroLineColor, ...
    'LineWidth', 1.0, ...
    'HandleVisibility', 'off');

grid(ax3, 'on');

xlabel(ax3, 'Trajectory point');
ylabel(ax3, 'Acceleration [m/s^2]');
title(ax3, 'Acceleration reference');

applyDarkStyle(ax3);

% Use the same horizontal limits for all three plots.
xLimits = [min(trajectoryPoint), max(trajectoryPoint)];

if xLimits(1) == xLimits(2)
    xLimits = xLimits + [-0.5 0.5];
end

xlim(ax1, xLimits);
xlim(ax2, xLimits);
xlim(ax3, xLimits);

linkaxes([ax1 ax2 ax3], 'x');

%% Figure 2: references spatially along the path

pathFigure = figure( ...
    'Name', 'Controller references along path', ...
    'Color', figureBackground, ...
    'Position', [150 60 1150 850]);

tiledlayout(2, 2, ...
    'TileSpacing', 'compact', ...
    'Padding', 'compact');

colormap(pathFigure, turbo(256));

ax4 = nexttile;
plotPathReference( ...
    ax4, x, y, frontReference, ...
    'Front steering reference', ...
    'Front steering [rad]');

ax5 = nexttile;
plotPathReference( ...
    ax5, x, y, rearReference, ...
    'Rear steering reference', ...
    'Rear steering [rad]');

ax6 = nexttile;
plotPathReference( ...
    ax6, x, y, velocityReference, ...
    'Velocity reference', ...
    'Velocity [m/s]');

ax7 = nexttile;
plotPathReference( ...
    ax7, x, y, accelerationReference, ...
    'Acceleration reference', ...
    'Acceleration [m/s^2]');

%% Print reference ranges in the Command Window

fprintf('\nController-reference snapshot %d\n', kIdx);
fprintf('Number of trajectory points: %d\n', numel(trajectoryPoint));

fprintf('Front steering: %+.4f to %+.4f rad\n', ...
    min(frontReference), max(frontReference));

fprintf('Rear steering : %+.4f to %+.4f rad\n', ...
    min(rearReference), max(rearReference));

fprintf('Velocity      : %+.4f to %+.4f m/s\n', ...
    min(velocityReference), max(velocityReference));

fprintf('Acceleration  : %+.4f to %+.4f m/s^2\n\n', ...
    min(accelerationReference), max(accelerationReference));

end


function plotPathReference(ax, x, y, values, plotTitle, colorbarLabel)
%PLOTPATHREFERENCE Draw one path-colored reference plot.

hold(ax, 'on');

% Draw a subtle path underneath the colored points.
plot(ax, x, y, ...
    '-', ...
    'Color', [0.45 0.45 0.45], ...
    'LineWidth', 1.0);

scatter(ax, x, y, ...
    34, values, ...
    'filled');

% Mark the first trajectory point.
plot(ax, x(1), y(1), ...
    'o', ...
    'MarkerSize', 8, ...
    'MarkerFaceColor', [1 1 1], ...
    'MarkerEdgeColor', [0 0 0], ...
    'LineWidth', 1.2);

axis(ax, 'equal');
grid(ax, 'on');
box(ax, 'on');

xlabel(ax, 'x [m]');
ylabel(ax, 'y [m]');
title(ax, plotTitle);

applyDarkStyle(ax);

cb = colorbar(ax);
cb.Label.String = colorbarLabel;
cb.Color = [0.92 0.92 0.92];
cb.Label.Color = [0.92 0.92 0.92];
cb.FontSize = 10;

end


function applyDarkStyle(ax)
%APPLYDARKSTYLE Apply a uniform dark theme to one axes.

backgroundColor = [0.055 0.055 0.055];
foregroundColor = [0.92 0.92 0.92];
gridColor       = [0.65 0.65 0.65];

ax.Color = backgroundColor;

ax.XColor = foregroundColor;
ax.YColor = foregroundColor;

ax.GridColor = gridColor;
ax.GridAlpha = 0.28;

ax.MinorGridColor = gridColor;
ax.MinorGridAlpha = 0.12;

ax.Title.Color = foregroundColor;
ax.XLabel.Color = foregroundColor;
ax.YLabel.Color = foregroundColor;

ax.Box = 'on';
ax.LineWidth = 0.8;
ax.FontSize = 10;

end


function applyDarkLegendStyle(lgd)
%APPLYDARKLEGENDSTYLE Apply the dark theme to a legend.

lgd.Color = [0.055 0.055 0.055];
lgd.TextColor = [0.92 0.92 0.92];
lgd.EdgeColor = [0.55 0.55 0.55];
lgd.FontSize = 9;

end