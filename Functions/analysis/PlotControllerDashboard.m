function dashboardFigure = PlotControllerDashboard( ...
    referenceLog, trackingLog, kIdx)
%PLOTCONTROLLERDASHBOARD Display all reference and tracking plots
% inside one reusable MATLAB figure tab.
%
% Reference log columns:
%   1: x
%   2: y
%   3: front steering reference
%   4: rear steering reference
%   5: velocity reference
%   6: acceleration reference
%
% Tracking log columns:
%   1: time
%   2: front reference
%   3: front controller command
%   4: delayed front command
%   5: actual front steering
%   6: rear reference
%   7: rear controller command
%   8: delayed rear command
%   9: actual rear steering
%  10: velocity reference
%  11: actual velocity
%  12: acceleration reference
%  13: acceleration controller command
%  14: delayed acceleration command
%  15: actual acceleration
%
% Usage from StopFcn:
%   PlotControllerDashboard( ...
%       controller_reference_log, ...
%       controller_tracking_log);

if nargin < 3
    kIdx = [];
end

%% Extract data

[reference, kIdx] = extractReferenceSnapshot(referenceLog, kIdx);
tracking = extractTrackingLog(trackingLog);

%% Calculate tracking differences

frontDifferences = [ ...
    tracking.frontCommand - tracking.frontReference, ...
    tracking.frontDelayed - tracking.frontCommand, ...
    tracking.frontActual - tracking.frontDelayed, ...
    tracking.frontActual - tracking.frontReference];

rearDifferences = [ ...
    tracking.rearCommand - tracking.rearReference, ...
    tracking.rearDelayed - tracking.rearCommand, ...
    tracking.rearActual - tracking.rearDelayed, ...
    tracking.rearActual - tracking.rearReference];

velocityDifference = ...
    tracking.velocityActual - tracking.velocityReference;

accelerationDifferences = [ ...
    tracking.accelerationCommand - tracking.accelerationReference, ...
    tracking.accelerationDelayed - tracking.accelerationCommand, ...
    tracking.accelerationActual - tracking.accelerationDelayed, ...
    tracking.accelerationActual - tracking.accelerationReference];

%% Colors

figureBackground = [0.025 0.025 0.025];

frontColor       = [0.20 0.55 1.00];
rearColor        = [1.00 0.25 0.25];
velocityColor    = [0.20 0.90 0.45];
accelerationColor = [1.00 0.45 0.12];

referenceColor = [0.15 0.75 1.00];
commandColor   = [1.00 0.65 0.10];
delayedColor   = [0.85 0.35 1.00];
actualColor    = [0.20 1.00 0.45];

stageColors = [ ...
    referenceColor;
    commandColor;
    delayedColor;
    actualColor];

stageStyles = {'-', '--', '-.', '-'};

differenceColors = [ ...
    commandColor;
    delayedColor;
    actualColor;
    0.95 0.95 0.95];

differenceStyles = {'-', '-', '-', '-'};

%% Create or reuse one figure tab

dashboardFigure = findall( ...
    groot, ...
    'Type', 'figure', ...
    'Tag', 'ATOMControllerDashboard');

if isempty(dashboardFigure)

    dashboardFigure = figure( ...
        'Name', 'ATOM Controller Dashboard', ...
        'NumberTitle', 'off', ...
        'Tag', 'ATOMControllerDashboard', ...
        'Color', figureBackground, ...
        'WindowStyle', 'docked', ...
        'InvertHardcopy', 'off');

else

    dashboardFigure = dashboardFigure(1);

    clf(dashboardFigure);

    set(dashboardFigure, ...
        'Name', 'ATOM Controller Dashboard', ...
        'NumberTitle', 'off', ...
        'Color', figureBackground, ...
        'InvertHardcopy', 'off');

    figure(dashboardFigure);

end

dashboardLayout = tiledlayout( ...
    dashboardFigure, 4, 4, ...
    'TileSpacing', 'compact', ...
    'Padding', 'compact');

dashboardTitle = title( ...
    dashboardLayout, ...
    sprintf('ATOM Controller Dashboard - Reference Snapshot %d', kIdx));

dashboardTitle.Color = [0.95 0.95 0.95];
dashboardTitle.FontWeight = 'bold';

colormap(dashboardFigure, turbo(256));

%% Row 1: references versus trajectory point

% Tile 1: steering references
ax1 = nexttile(dashboardLayout, 1);

plot(ax1, ...
    reference.point, reference.frontSteering, ...
    '-', ...
    'Color', frontColor, ...
    'LineWidth', 1.7);

hold(ax1, 'on');

plot(ax1, ...
    reference.point, reference.rearSteering, ...
    '-', ...
    'Color', rearColor, ...
    'LineWidth', 1.7);

drawZeroLine(ax1);

xlabel(ax1, 'Trajectory point');
ylabel(ax1, 'Angle [rad]');
title(ax1, 'Steering references');

lgd1 = legend(ax1, ...
    {'Front', 'Rear'}, ...
    'Location', 'best');

applyDarkLegendStyle(lgd1);
applyDarkAxesStyle(ax1);

% Tile 2: velocity reference
ax2 = nexttile(dashboardLayout, 2);

plot(ax2, ...
    reference.point, reference.velocity, ...
    '-', ...
    'Color', velocityColor, ...
    'LineWidth', 1.7);

drawZeroLine(ax2);

xlabel(ax2, 'Trajectory point');
ylabel(ax2, 'Velocity [m/s]');
title(ax2, 'Velocity reference');

applyDarkAxesStyle(ax2);

% Tile 3: acceleration reference
ax3 = nexttile(dashboardLayout, 3);

plot(ax3, ...
    reference.point, reference.acceleration, ...
    '-', ...
    'Color', accelerationColor, ...
    'LineWidth', 1.7);

drawZeroLine(ax3);

xlabel(ax3, 'Trajectory point');
ylabel(ax3, 'Acceleration [m/s^2]');
title(ax3, 'Acceleration reference');

applyDarkAxesStyle(ax3);

% Tile 4: numerical summary
ax4 = nexttile(dashboardLayout, 4);

showSummary( ...
    ax4, reference, tracking, ...
    frontDifferences, ...
    rearDifferences, ...
    velocityDifference, ...
    accelerationDifferences, ...
    kIdx);

%% Row 2: references spatially along the path

ax5 = nexttile(dashboardLayout, 5);

plotPathReference( ...
    ax5, ...
    reference.x, ...
    reference.y, ...
    reference.frontSteering, ...
    'Front steering along path', ...
    'Front [rad]');

ax6 = nexttile(dashboardLayout, 6);

plotPathReference( ...
    ax6, ...
    reference.x, ...
    reference.y, ...
    reference.rearSteering, ...
    'Rear steering along path', ...
    'Rear [rad]');

ax7 = nexttile(dashboardLayout, 7);

plotPathReference( ...
    ax7, ...
    reference.x, ...
    reference.y, ...
    reference.velocity, ...
    'Velocity along path', ...
    'Velocity [m/s]');

ax8 = nexttile(dashboardLayout, 8);

plotPathReference( ...
    ax8, ...
    reference.x, ...
    reference.y, ...
    reference.acceleration, ...
    'Acceleration along path', ...
    'Acceleration [m/s^2]');

%% Row 3: time-domain tracking

trackingLabels = { ...
    'Reference', ...
    'Controller command', ...
    'After delay', ...
    'Vehicle output'};

% Tile 9: front steering tracking
ax9 = nexttile(dashboardLayout, 9);

frontStages = [ ...
    tracking.frontReference, ...
    tracking.frontCommand, ...
    tracking.frontDelayed, ...
    tracking.frontActual];

plotMultipleSignals( ...
    ax9, tracking.time, frontStages, ...
    stageColors, stageStyles);

drawZeroLine(ax9);

xlabel(ax9, 'Time [s]');
ylabel(ax9, 'Angle [rad]');
title(ax9, 'Front steering tracking');

lgd9 = legend(ax9, trackingLabels, 'Location', 'best');

applyDarkLegendStyle(lgd9);
applyDarkAxesStyle(ax9);

% Tile 10: rear steering tracking
ax10 = nexttile(dashboardLayout, 10);

rearStages = [ ...
    tracking.rearReference, ...
    tracking.rearCommand, ...
    tracking.rearDelayed, ...
    tracking.rearActual];

plotMultipleSignals( ...
    ax10, tracking.time, rearStages, ...
    stageColors, stageStyles);

drawZeroLine(ax10);

xlabel(ax10, 'Time [s]');
ylabel(ax10, 'Angle [rad]');
title(ax10, 'Rear steering tracking');

lgd10 = legend(ax10, trackingLabels, 'Location', 'best');

applyDarkLegendStyle(lgd10);
applyDarkAxesStyle(ax10);

% Tile 11: velocity tracking
ax11 = nexttile(dashboardLayout, 11);

velocityStages = [ ...
    tracking.velocityReference, ...
    tracking.velocityActual];

velocityStageColors = [ ...
    referenceColor;
    actualColor];

velocityStageStyles = {'-', '-'};

plotMultipleSignals( ...
    ax11, tracking.time, velocityStages, ...
    velocityStageColors, velocityStageStyles);

drawZeroLine(ax11);

xlabel(ax11, 'Time [s]');
ylabel(ax11, 'Velocity [m/s]');
title(ax11, 'Velocity tracking');

lgd11 = legend(ax11, ...
    {'Reference', 'Vehicle output'}, ...
    'Location', 'best');

applyDarkLegendStyle(lgd11);
applyDarkAxesStyle(ax11);

% Tile 12: acceleration tracking
ax12 = nexttile(dashboardLayout, 12);

accelerationStages = [ ...
    tracking.accelerationReference, ...
    tracking.accelerationCommand, ...
    tracking.accelerationDelayed, ...
    tracking.accelerationActual];

plotMultipleSignals( ...
    ax12, tracking.time, accelerationStages, ...
    stageColors, stageStyles);

drawZeroLine(ax12);

xlabel(ax12, 'Time [s]');
ylabel(ax12, 'Acceleration [m/s^2]');
title(ax12, 'Acceleration tracking');

lgd12 = legend(ax12, trackingLabels, 'Location', 'best');

applyDarkLegendStyle(lgd12);
applyDarkAxesStyle(ax12);

%% Row 4: differences between stages

differenceLabels = { ...
    'Command - reference', ...
    'Delayed - command', ...
    'Actual - delayed', ...
    'Actual - reference'};

% Tile 13: front steering differences
ax13 = nexttile(dashboardLayout, 13);

plotMultipleSignals( ...
    ax13, tracking.time, frontDifferences, ...
    differenceColors, differenceStyles);

drawZeroLine(ax13);

xlabel(ax13, 'Time [s]');
ylabel(ax13, 'Difference [rad]');
title(ax13, 'Front steering differences');

lgd13 = legend(ax13, differenceLabels, 'Location', 'best');

applyDarkLegendStyle(lgd13);
applyDarkAxesStyle(ax13);

% Tile 14: rear steering differences
ax14 = nexttile(dashboardLayout, 14);

plotMultipleSignals( ...
    ax14, tracking.time, rearDifferences, ...
    differenceColors, differenceStyles);

drawZeroLine(ax14);

xlabel(ax14, 'Time [s]');
ylabel(ax14, 'Difference [rad]');
title(ax14, 'Rear steering differences');

lgd14 = legend(ax14, differenceLabels, 'Location', 'best');

applyDarkLegendStyle(lgd14);
applyDarkAxesStyle(ax14);

% Tile 15: velocity difference
ax15 = nexttile(dashboardLayout, 15);

plot(ax15, ...
    tracking.time, velocityDifference, ...
    '-', ...
    'Color', [0.95 0.95 0.95], ...
    'LineWidth', 1.7);

drawZeroLine(ax15);

xlabel(ax15, 'Time [s]');
ylabel(ax15, 'Error [m/s]');
title(ax15, 'Velocity: actual - reference');

lgd15 = legend(ax15, ...
    {'Actual - reference'}, ...
    'Location', 'best');

applyDarkLegendStyle(lgd15);
applyDarkAxesStyle(ax15);

% Tile 16: acceleration differences
ax16 = nexttile(dashboardLayout, 16);

plotMultipleSignals( ...
    ax16, tracking.time, accelerationDifferences, ...
    differenceColors, differenceStyles);

drawZeroLine(ax16);

xlabel(ax16, 'Time [s]');
ylabel(ax16, 'Difference [m/s^2]');
title(ax16, 'Acceleration differences');

lgd16 = legend(ax16, differenceLabels, 'Location', 'best');

applyDarkLegendStyle(lgd16);
applyDarkAxesStyle(ax16);

%% Link all time-domain plots

linkaxes( ...
    [ax9 ax10 ax11 ax12 ax13 ax14 ax15 ax16], ...
    'x');

drawnow;

end


function [reference, kIdx] = extractReferenceSnapshot(logData, kIdx)

data = unwrapLogData(logData);
data = double(data);

if ndims(data) == 3

    if size(data,2) ~= 6
        error('PlotControllerDashboard:InvalidReferenceLog', ...
            ['Expected reference dimensions [points x 6 x snapshots], ' ...
             'but received %s.'], mat2str(size(data)));
    end

    numberOfSnapshots = size(data,3);

    if isempty(kIdx)

        snapshotHasData = ...
            squeeze(any(any(abs(data) > 0, 1), 2));

        kIdx = find(snapshotHasData, 1, 'last');

        if isempty(kIdx)
            error('Reference log contains only zeros.');
        end
    end

    if kIdx < 1 || ...
       kIdx > numberOfSnapshots || ...
       kIdx ~= floor(kIdx)

        error('Snapshot index must be an integer from 1 to %d.', ...
            numberOfSnapshots);
    end

    matrix = data(:,:,kIdx);

elseif ismatrix(data) && size(data,2) == 6

    matrix = data;
    kIdx = 1;

elseif ismatrix(data) && size(data,1) == 6

    matrix = data.';
    kIdx = 1;

else

    error('PlotControllerDashboard:InvalidReferenceLog', ...
        'Unexpected reference-log size %s.', mat2str(size(data)));
end

valid = all(isfinite(matrix), 2);
matrix = matrix(valid,:);

if isempty(matrix)
    error('The selected reference snapshot contains no valid points.');
end

reference.x = matrix(:,1);
reference.y = matrix(:,2);

reference.frontSteering = matrix(:,3);
reference.rearSteering  = matrix(:,4);

reference.velocity     = matrix(:,5);
reference.acceleration = matrix(:,6);

reference.point = (1:size(matrix,1)).';

end


function tracking = extractTrackingLog(logData)

data = unwrapLogData(logData);
data = squeeze(double(data));

if size(data,2) ~= 15 && size(data,1) == 15
    data = data.';
end

if size(data,2) ~= 15
    error('PlotControllerDashboard:InvalidTrackingLog', ...
        ['Expected tracking dimensions [samples x 15], ' ...
         'but received %s.'], mat2str(size(data)));
end

data = data(isfinite(data(:,1)),:);

if isempty(data)
    error('The tracking log contains no valid samples.');
end

tracking.time = data(:,1);

tracking.frontReference = data(:,2);
tracking.frontCommand   = data(:,3);
tracking.frontDelayed   = data(:,4);
tracking.frontActual    = data(:,5);

tracking.rearReference = data(:,6);
tracking.rearCommand   = data(:,7);
tracking.rearDelayed   = data(:,8);
tracking.rearActual    = data(:,9);

tracking.velocityReference = data(:,10);
tracking.velocityActual    = data(:,11);

tracking.accelerationReference = data(:,12);
tracking.accelerationCommand   = data(:,13);
tracking.accelerationDelayed   = data(:,14);
tracking.accelerationActual    = data(:,15);

end


function data = unwrapLogData(data)

if isa(data, 'Simulink.SimulationData.Signal')
    data = data.Values;
end

if isa(data, 'timeseries')
    data = data.Data;

elseif isstruct(data) && isfield(data, 'signals')
    data = data.signals.values;
end

end


function plotMultipleSignals( ...
    ax, xData, yData, colors, lineStyles)

hold(ax, 'on');

numberOfSignals = size(yData,2);

for signalIndex = 1:numberOfSignals

    plot(ax, ...
        xData, ...
        yData(:,signalIndex), ...
        lineStyles{signalIndex}, ...
        'Color', colors(signalIndex,:), ...
        'LineWidth', 1.5);
end

grid(ax, 'on');
box(ax, 'on');

end


function plotPathReference( ...
    ax, x, y, values, plotTitle, colorbarLabel)

hold(ax, 'on');

plot(ax, ...
    x, y, ...
    '-', ...
    'Color', [0.45 0.45 0.45], ...
    'LineWidth', 0.8);

scatter(ax, ...
    x, y, ...
    24, values, ...
    'filled');

plot(ax, ...
    x(1), y(1), ...
    'o', ...
    'MarkerSize', 6, ...
    'MarkerFaceColor', [1 1 1], ...
    'MarkerEdgeColor', [0 0 0], ...
    'LineWidth', 1.0);

axis(ax, 'equal');
grid(ax, 'on');
box(ax, 'on');

xlabel(ax, 'x [m]');
ylabel(ax, 'y [m]');
title(ax, plotTitle);

applyDarkAxesStyle(ax);

cb = colorbar(ax);
cb.Label.String = colorbarLabel;
cb.Color = [0.92 0.92 0.92];
cb.Label.Color = [0.92 0.92 0.92];
cb.FontSize = 7;

end


function showSummary( ...
    ax, reference, tracking, ...
    frontDifferences, rearDifferences, ...
    velocityDifference, accelerationDifferences, ...
    kIdx)

ax.Color = [0.055 0.055 0.055];
axis(ax, 'off');

frontMaximum = max(abs(frontDifferences(:,4)), [], 'omitnan');
rearMaximum = max(abs(rearDifferences(:,4)), [], 'omitnan');

velocityMaximum = ...
    max(abs(velocityDifference), [], 'omitnan');

accelerationMaximum = ...
    max(abs(accelerationDifferences(:,4)), [], 'omitnan');

summaryText = sprintf([ ...
    'REFERENCE SNAPSHOT\n' ...
    'Snapshot: %d\n' ...
    'Path points: %d\n\n' ...
    'REFERENCE RANGES\n' ...
    'Front: %+.3f to %+.3f rad\n' ...
    'Rear : %+.3f to %+.3f rad\n' ...
    'Vel. : %+.3f to %+.3f m/s\n' ...
    'Acc. : %+.3f to %+.3f m/s^2\n\n' ...
    'MAX |ACTUAL - REFERENCE|\n' ...
    'Front: %.4f rad\n' ...
    'Rear : %.4f rad\n' ...
    'Vel. : %.4f m/s\n' ...
    'Acc. : %.4f m/s^2\n\n' ...
    'Simulation time: %.2f s'], ...
    kIdx, ...
    numel(reference.point), ...
    min(reference.frontSteering), ...
    max(reference.frontSteering), ...
    min(reference.rearSteering), ...
    max(reference.rearSteering), ...
    min(reference.velocity), ...
    max(reference.velocity), ...
    min(reference.acceleration), ...
    max(reference.acceleration), ...
    frontMaximum, ...
    rearMaximum, ...
    velocityMaximum, ...
    accelerationMaximum, ...
    tracking.time(end) - tracking.time(1));

text(ax, ...
    0.03, 0.97, ...
    summaryText, ...
    'Units', 'normalized', ...
    'VerticalAlignment', 'top', ...
    'HorizontalAlignment', 'left', ...
    'Color', [0.92 0.92 0.92], ...
    'FontName', 'monospaced', ...
    'FontSize', 7.5);

end


function drawZeroLine(ax)

hold(ax, 'on');

yline(ax, ...
    0, ':', ...
    'Color', [0.65 0.65 0.65], ...
    'LineWidth', 0.8, ...
    'HandleVisibility', 'off');

end


function applyDarkAxesStyle(ax)

backgroundColor = [0.055 0.055 0.055];
foregroundColor = [0.92 0.92 0.92];
gridColor = [0.65 0.65 0.65];

ax.Color = backgroundColor;

ax.XColor = foregroundColor;
ax.YColor = foregroundColor;

ax.GridColor = gridColor;
ax.GridAlpha = 0.24;

ax.MinorGridColor = gridColor;
ax.MinorGridAlpha = 0.10;

ax.Title.Color = foregroundColor;
ax.XLabel.Color = foregroundColor;
ax.YLabel.Color = foregroundColor;

ax.Box = 'on';
ax.LineWidth = 0.7;
ax.FontSize = 7.5;

end


function applyDarkLegendStyle(lgd)

lgd.Color = [0.055 0.055 0.055];
lgd.TextColor = [0.92 0.92 0.92];
lgd.EdgeColor = [0.50 0.50 0.50];
lgd.FontSize = 6.5;
lgd.AutoUpdate = 'off';

end