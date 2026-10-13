function PlotControllerTracking(logData)
%PLOTCONTROLLERTRACKING Compare references, controller commands,
% delayed commands, and measured vehicle-model outputs.
%
% Expected column order:
%   1  time
%   2  front steering reference
%   3  front controller command
%   4  delayed front command
%   5  actual front steering
%   6  rear steering reference
%   7  rear controller command
%   8  delayed rear command
%   9  actual rear steering
%   10 velocity reference
%   11 actual velocity
%   12 acceleration reference
%   13 acceleration controller command
%   14 delayed acceleration command
%   15 actual acceleration
%
% Example:
%   PlotControllerTracking(out.controller_tracking_log)

%% Unwrap Simulink logging formats

if isa(logData, 'Simulink.SimulationData.Signal')
    logData = logData.Values;
end

if isa(logData, 'timeseries')
    logData = logData.Data;

elseif isstruct(logData) && isfield(logData, 'signals')
    logData = logData.signals.values;
end

data = squeeze(double(logData));

% Accept either [samples x 15] or [15 x samples].
if size(data, 2) ~= 15 && size(data, 1) == 15
    data = data.';
end

if size(data, 2) ~= 15
    error('PlotControllerTracking:InvalidLog', ...
        ['Expected 15 logged columns, but received an array ' ...
         'with size %s.'], mat2str(size(data)));
end

% Remove samples with invalid timestamps.
data = data(isfinite(data(:,1)), :);

if isempty(data)
    error('PlotControllerTracking:EmptyLog', ...
        'The tracking log does not contain valid samples.');
end

%% Extract signals

time = data(:,1);

frontReference = data(:,2);
frontCommand   = data(:,3);
frontDelayed   = data(:,4);
frontActual    = data(:,5);

rearReference = data(:,6);
rearCommand   = data(:,7);
rearDelayed   = data(:,8);
rearActual    = data(:,9);

velocityReference = data(:,10);
velocityActual    = data(:,11);

accelerationReference = data(:,12);
accelerationCommand   = data(:,13);
accelerationDelayed   = data(:,14);
accelerationActual    = data(:,15);

%% Dark-theme colors

backgroundColor = [0.025 0.025 0.025];

referenceColor = [0.15 0.75 1.00];
commandColor   = [1.00 0.65 0.10];
delayedColor   = [0.85 0.35 1.00];
actualColor    = [0.20 1.00 0.45];

%% Figure 1: signal-stage comparisons

figure( ...
    'Name', 'Controller tracking comparison', ...
    'Color', backgroundColor, ...
    'Position', [80 50 1200 900], ...
    'InvertHardcopy', 'off');

tiledlayout(4, 1, ...
    'TileSpacing', 'compact', ...
    'Padding', 'compact');

% Front steering
ax1 = nexttile;

plotFourStages( ...
    ax1, time, ...
    frontReference, frontCommand, frontDelayed, frontActual, ...
    referenceColor, commandColor, delayedColor, actualColor);

ylabel(ax1, 'Front steering [rad]');
title(ax1, 'Front steering tracking');

lgd1 = legend(ax1, ...
    {'Reference', 'Controller command', ...
     'After delay', 'Vehicle output'}, ...
    'Location', 'best');

applyDarkLegendStyle(lgd1);
applyDarkAxesStyle(ax1);

% Rear steering
ax2 = nexttile;

plotFourStages( ...
    ax2, time, ...
    rearReference, rearCommand, rearDelayed, rearActual, ...
    referenceColor, commandColor, delayedColor, actualColor);

ylabel(ax2, 'Rear steering [rad]');
title(ax2, 'Rear steering tracking');

lgd2 = legend(ax2, ...
    {'Reference', 'Controller command', ...
     'After delay', 'Vehicle output'}, ...
    'Location', 'best');

applyDarkLegendStyle(lgd2);
applyDarkAxesStyle(ax2);

% Velocity
ax3 = nexttile;

plot(ax3, time, velocityReference, ...
    '-', 'Color', referenceColor, 'LineWidth', 1.8);

hold(ax3, 'on');

plot(ax3, time, velocityActual, ...
    '-', 'Color', actualColor, 'LineWidth', 1.8);

grid(ax3, 'on');
drawZeroLine(ax3);

ylabel(ax3, 'Velocity [m/s]');
title(ax3, 'Longitudinal velocity tracking');

lgd3 = legend(ax3, ...
    {'Reference', 'Vehicle output'}, ...
    'Location', 'best');

applyDarkLegendStyle(lgd3);
applyDarkAxesStyle(ax3);

% Acceleration
ax4 = nexttile;

plotFourStages( ...
    ax4, time, ...
    accelerationReference, ...
    accelerationCommand, ...
    accelerationDelayed, ...
    accelerationActual, ...
    referenceColor, commandColor, delayedColor, actualColor);

xlabel(ax4, 'Time [s]');
ylabel(ax4, 'Acceleration [m/s^2]');
title(ax4, 'Longitudinal acceleration tracking');

lgd4 = legend(ax4, ...
    {'Reference', 'Controller command', ...
     'After delay', 'Vehicle output'}, ...
    'Location', 'best');

applyDarkLegendStyle(lgd4);
applyDarkAxesStyle(ax4);

linkaxes([ax1 ax2 ax3 ax4], 'x');

sg = sgtitle('Reference, command, delay, and vehicle response');
sg.Color = [0.95 0.95 0.95];
sg.FontWeight = 'bold';

%% Calculate differences

frontCommandError = frontCommand - frontReference;
frontDelayChange  = frontDelayed - frontCommand;
frontVehicleError = frontActual - frontDelayed;
frontTotalError   = frontActual - frontReference;

rearCommandError = rearCommand - rearReference;
rearDelayChange  = rearDelayed - rearCommand;
rearVehicleError = rearActual - rearDelayed;
rearTotalError   = rearActual - rearReference;

velocityTotalError = velocityActual - velocityReference;

accelerationCommandError = ...
    accelerationCommand - accelerationReference;

accelerationDelayChange = ...
    accelerationDelayed - accelerationCommand;

accelerationVehicleError = ...
    accelerationActual - accelerationDelayed;

accelerationTotalError = ...
    accelerationActual - accelerationReference;

%% Figure 2: differences between stages

figure( ...
    'Name', 'Controller tracking differences', ...
    'Color', backgroundColor, ...
    'Position', [130 70 1200 900], ...
    'InvertHardcopy', 'off');

tiledlayout(4, 1, ...
    'TileSpacing', 'compact', ...
    'Padding', 'compact');

% Front steering differences
ax5 = nexttile;

plotDifferenceStages( ...
    ax5, time, ...
    frontCommandError, ...
    frontDelayChange, ...
    frontVehicleError, ...
    frontTotalError);

ylabel(ax5, 'Difference [rad]');
title(ax5, 'Front steering differences');

lgd5 = legend(ax5, ...
    {'Command - reference', ...
     'Delayed - command', ...
     'Actual - delayed', ...
     'Actual - reference'}, ...
    'Location', 'best');

applyDarkLegendStyle(lgd5);
applyDarkAxesStyle(ax5);

% Rear steering differences
ax6 = nexttile;

plotDifferenceStages( ...
    ax6, time, ...
    rearCommandError, ...
    rearDelayChange, ...
    rearVehicleError, ...
    rearTotalError);

ylabel(ax6, 'Difference [rad]');
title(ax6, 'Rear steering differences');

lgd6 = legend(ax6, ...
    {'Command - reference', ...
     'Delayed - command', ...
     'Actual - delayed', ...
     'Actual - reference'}, ...
    'Location', 'best');

applyDarkLegendStyle(lgd6);
applyDarkAxesStyle(ax6);

% Velocity difference
ax7 = nexttile;

plot(ax7, time, velocityTotalError, ...
    '-', ...
    'Color', [0.95 0.95 0.95], ...
    'LineWidth', 1.8);

grid(ax7, 'on');
drawZeroLine(ax7);

ylabel(ax7, 'Error [m/s]');
title(ax7, 'Velocity error: actual - reference');

lgd7 = legend(ax7, {'Actual - reference'}, ...
    'Location', 'best');

applyDarkLegendStyle(lgd7);
applyDarkAxesStyle(ax7);

% Acceleration differences
ax8 = nexttile;

plotDifferenceStages( ...
    ax8, time, ...
    accelerationCommandError, ...
    accelerationDelayChange, ...
    accelerationVehicleError, ...
    accelerationTotalError);

xlabel(ax8, 'Time [s]');
ylabel(ax8, 'Difference [m/s^2]');
title(ax8, 'Acceleration differences');

lgd8 = legend(ax8, ...
    {'Command - reference', ...
     'Delayed - command', ...
     'Actual - delayed', ...
     'Actual - reference'}, ...
    'Location', 'best');

applyDarkLegendStyle(lgd8);
applyDarkAxesStyle(ax8);

linkaxes([ax5 ax6 ax7 ax8], 'x');

sg = sgtitle('Differences between signal stages');
sg.Color = [0.95 0.95 0.95];
sg.FontWeight = 'bold';

%% Print maximum absolute total errors

fprintf('\nMaximum absolute tracking errors\n');
fprintf('Front steering : %.5f rad\n', ...
    max(abs(frontTotalError)));

fprintf('Rear steering  : %.5f rad\n', ...
    max(abs(rearTotalError)));

fprintf('Velocity       : %.5f m/s\n', ...
    max(abs(velocityTotalError)));

fprintf('Acceleration   : %.5f m/s^2\n\n', ...
    max(abs(accelerationTotalError)));

end


function plotFourStages( ...
    ax, time, reference, command, delayed, actual, ...
    referenceColor, commandColor, delayedColor, actualColor)

plot(ax, time, reference, ...
    '-', 'Color', referenceColor, 'LineWidth', 1.8);

hold(ax, 'on');

plot(ax, time, command, ...
    '--', 'Color', commandColor, 'LineWidth', 1.6);

plot(ax, time, delayed, ...
    '-.', 'Color', delayedColor, 'LineWidth', 1.6);

plot(ax, time, actual, ...
    '-', 'Color', actualColor, 'LineWidth', 1.8);

grid(ax, 'on');
drawZeroLine(ax);

end


function plotDifferenceStages( ...
    ax, time, commandError, delayChange, vehicleError, totalError)

plot(ax, time, commandError, ...
    '-', ...
    'Color', [1.00 0.65 0.10], ...
    'LineWidth', 1.5);

hold(ax, 'on');

plot(ax, time, delayChange, ...
    '-', ...
    'Color', [0.85 0.35 1.00], ...
    'LineWidth', 1.5);

plot(ax, time, vehicleError, ...
    '-', ...
    'Color', [0.20 1.00 0.45], ...
    'LineWidth', 1.5);

plot(ax, time, totalError, ...
    '-', ...
    'Color', [0.95 0.95 0.95], ...
    'LineWidth', 2.0);

grid(ax, 'on');
drawZeroLine(ax);

end


function drawZeroLine(ax)

yline(ax, 0, ':', ...
    'Color', [0.65 0.65 0.65], ...
    'LineWidth', 1.0, ...
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
ax.GridAlpha = 0.25;

ax.MinorGridColor = gridColor;
ax.MinorGridAlpha = 0.10;

ax.Title.Color = foregroundColor;
ax.XLabel.Color = foregroundColor;
ax.YLabel.Color = foregroundColor;

ax.Box = 'on';
ax.LineWidth = 0.8;
ax.FontSize = 10;

end


function applyDarkLegendStyle(lgd)

lgd.Color = [0.055 0.055 0.055];
lgd.TextColor = [0.92 0.92 0.92];
lgd.EdgeColor = [0.55 0.55 0.55];
lgd.FontSize = 9;

end