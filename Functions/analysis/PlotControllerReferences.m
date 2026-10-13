function PlotControllerReferences(logData, segmentSnapshots)
% PlotControllerReferences
%
% Plots the complete controller-reference trajectory across all maneuver
% segments.
%
% Expected logger columns:
%   1: x position                       [m]
%   2: y position                       [m]
%   3: front steering reference         [rad]
%   4: rear steering reference          [rad]
%   5: longitudinal velocity reference  [m/s]
%   6: acceleration reference           [m/s^2]
%
% Automatic segment detection:
%   PlotControllerReferences(out.controller_reference_log)
%
% Optional manual segment snapshots:
%   PlotControllerReferences(out.controller_reference_log, [125 3836])

    if nargin < 2
        segmentSnapshots = [];
    end

    %% Plot colours

    figureBackground   = [0.00 0.00 0.00];
    axesBackground     = [0.055 0.055 0.055];
    foreground         = [0.88 0.88 0.88];
    gridColour         = [0.38 0.38 0.38];

    frontColour        = [0.10 0.35 1.00];
    rearColour         = [1.00 0.10 0.10];
    velocityColour     = [0.05 0.80 0.35];
    accelerationColour = [1.00 0.35 0.05];

    %% Extract the numerical logging array

    logData = unwrapReferenceLog(logData);

    if ~isnumeric(logData)
        error(['The controller-reference log must contain a ', ...
               'numerical array.']);
    end

    logData = double(logData);

    if isempty(logData)
        error('The controller-reference log is empty.');
    end

    if size(logData,2) ~= 6 || ndims(logData) > 3
        error(['Expected reference data with dimensions ', ...
               '[trajectory points x 6 x snapshots].']);
    end

    numberOfSnapshots = size(logData,3);

    %% Select the best snapshot from every trajectory segment

    if isempty(segmentSnapshots)

        [segmentSnapshots, segmentDirections] = ...
            findSegmentSnapshots(logData);

    else

        validateattributes(segmentSnapshots, {'numeric'}, ...
            {'vector', 'integer', 'positive', ...
             '<=', numberOfSnapshots}, ...
             mfilename, 'segmentSnapshots');

        segmentSnapshots = unique( ...
            segmentSnapshots(:).', 'stable');

        segmentDirections = zeros(size(segmentSnapshots));

        for segmentIndex = 1:numel(segmentSnapshots)

            snapshot = cleanSnapshot( ...
                logData(:,:,segmentSnapshots(segmentIndex)));

            segmentDirections(segmentIndex) = ...
                findReferenceDirection(snapshot);
        end
    end

    if isempty(segmentSnapshots)
        error(['No valid reference trajectory was found in ', ...
               'controller_reference_log.']);
    end

    %% Extract the selected segments

    numberOfSegments = numel(segmentSnapshots);

    segments = cell(1, numberOfSegments);
    segmentLengths = zeros(1, numberOfSegments);

    for segmentIndex = 1:numberOfSegments

        snapshotIndex = segmentSnapshots(segmentIndex);

        currentSegment = cleanSnapshot( ...
            logData(:,:,snapshotIndex));

        if size(currentSegment,1) < 2
            error('Snapshot %d does not contain a valid path.', ...
                  snapshotIndex);
        end

        segments{segmentIndex} = currentSegment;

        segmentLengths(segmentIndex) = sum(hypot( ...
            diff(currentSegment(:,1)), ...
            diff(currentSegment(:,2))));
    end

    %% Orient adjoining segments consistently

    for segmentIndex = 2:numberOfSegments

        previousSegment = segments{segmentIndex - 1};
        currentSegment  = segments{segmentIndex};

        previousEndpoint = previousSegment(end,1:2);

        distanceToCurrentStart = norm( ...
            previousEndpoint - currentSegment(1,1:2));

        distanceToCurrentEnd = norm( ...
            previousEndpoint - currentSegment(end,1:2));

        % Reverse the point order when the end is closer to the
        % preceding segment than the beginning.
        if distanceToCurrentEnd < distanceToCurrentStart
            currentSegment = flipud(currentSegment);
        end

        segments{segmentIndex} = currentSegment;
    end

    %% Join the trajectory segments

    completeReference = [];

    segmentStart = zeros(1, numberOfSegments);
    segmentEnd   = zeros(1, numberOfSegments);

    for segmentIndex = 1:numberOfSegments

        currentSegment = segments{segmentIndex};

        if ~isempty(completeReference)

            joiningDistance = norm( ...
                completeReference(end,1:2) - ...
                currentSegment(1,1:2));

            joiningTolerance = calculateJoiningTolerance( ...
                completeReference, currentSegment);

            % Remove a duplicated segment junction.
            if joiningDistance <= joiningTolerance
                currentSegment(1,:) = [];
            end
        end

        if isempty(currentSegment)
            continue;
        end

        segmentStart(segmentIndex) = ...
            size(completeReference,1) + 1;

        completeReference = [
            completeReference
            currentSegment
        ]; %#ok<AGROW>

        segmentEnd(segmentIndex) = ...
            size(completeReference,1);
    end

    if isempty(completeReference)
        error('The combined reference trajectory is empty.');
    end

    %% Extract complete reference signals

    xPosition     = completeReference(:,1);
    yPosition     = completeReference(:,2);
    frontSteering = completeReference(:,3);
    rearSteering  = completeReference(:,4);
    velocity      = completeReference(:,5);
    acceleration  = completeReference(:,6);

    trajectoryPoint = (1:size(completeReference,1)).';

    totalPathLength = sum(segmentLengths);

    snapshotText = strtrim( ...
        sprintf('%d ', segmentSnapshots));

    %% Figure 1: references by complete trajectory point

    pointFigure = getDockedFigure( ...
        'ATOMReferencePointFigure', ...
        'Controller references by trajectory point', ...
        figureBackground);

    pointLayout = tiledlayout(pointFigure, 3, 1, ...
        'TileSpacing', 'compact', ...
        'Padding', 'compact');

    title(pointLayout, sprintf( ...
        'Complete controller-reference trajectory — %d segments', ...
        numberOfSegments), ...
        'Color', foreground, ...
        'FontWeight', 'bold');

    % Steering references
    steeringAxes = nexttile(pointLayout, 1);

    plot(steeringAxes, ...
        trajectoryPoint, frontSteering, ...
        'Color', frontColour, ...
        'LineWidth', 1.7, ...
        'DisplayName', 'Front reference');

    hold(steeringAxes, 'on');

    plot(steeringAxes, ...
        trajectoryPoint, rearSteering, ...
        'Color', rearColour, ...
        'LineWidth', 1.7, ...
        'DisplayName', 'Rear reference');

    yline(steeringAxes, 0, ':', ...
        'Color', [0.55 0.55 0.55], ...
        'HandleVisibility', 'off');

    addSegmentBoundaries(steeringAxes, segmentStart);

    title(steeringAxes, 'Steering references');
    ylabel(steeringAxes, 'Steering [rad]');
    xlabel(steeringAxes, 'Complete trajectory point');

    steeringLegend = legend(steeringAxes, ...
        'Location', 'best');

    applyDarkAxesStyle(steeringAxes, ...
        axesBackground, foreground, gridColour);

    applyDarkLegendStyle(steeringLegend, ...
        axesBackground, foreground);

    % Velocity reference
    velocityAxes = nexttile(pointLayout, 2);

    plot(velocityAxes, ...
        trajectoryPoint, velocity, ...
        'Color', velocityColour, ...
        'LineWidth', 1.7);

    hold(velocityAxes, 'on');

    yline(velocityAxes, 0, ':', ...
        'Color', [0.55 0.55 0.55], ...
        'HandleVisibility', 'off');

    addSegmentBoundaries(velocityAxes, segmentStart);

    title(velocityAxes, ...
        'Longitudinal velocity reference');

    ylabel(velocityAxes, 'Velocity [m/s]');
    xlabel(velocityAxes, 'Complete trajectory point');

    applyDarkAxesStyle(velocityAxes, ...
        axesBackground, foreground, gridColour);

    % Acceleration reference
    accelerationAxes = nexttile(pointLayout, 3);

    plot(accelerationAxes, ...
        trajectoryPoint, acceleration, ...
        'Color', accelerationColour, ...
        'LineWidth', 1.7);

    hold(accelerationAxes, 'on');

    yline(accelerationAxes, 0, ':', ...
        'Color', [0.55 0.55 0.55], ...
        'HandleVisibility', 'off');

    addSegmentBoundaries(accelerationAxes, segmentStart);

    title(accelerationAxes, 'Acceleration reference');
    ylabel(accelerationAxes, 'Acceleration [m/s^2]');
    xlabel(accelerationAxes, 'Complete trajectory point');

    applyDarkAxesStyle(accelerationAxes, ...
        axesBackground, foreground, gridColour);

    linkaxes( ...
        [steeringAxes, velocityAxes, accelerationAxes], 'x');

    xlim(steeringAxes, ...
        [1, max(2, numel(trajectoryPoint))]);

    %% Figure 2: reference values along the complete spatial path

    pathFigure = getDockedFigure( ...
        'ATOMReferencePathFigure', ...
        'Controller references along path', ...
        figureBackground);

    pathLayout = tiledlayout(pathFigure, 2, 2, ...
        'TileSpacing', 'compact', ...
        'Padding', 'compact');

    title(pathLayout, sprintf( ...
        ['Complete controller-reference path — %d segments, ' ...
         'selected snapshots: %s'], ...
        numberOfSegments, snapshotText), ...
        'Color', foreground, ...
        'FontWeight', 'bold');

    frontPathAxes = nexttile(pathLayout, 1);

    plotPathReference(frontPathAxes, ...
        xPosition, yPosition, frontSteering, ...
        segmentStart, segmentEnd, ...
        'Front steering reference', ...
        'Front steering [rad]', ...
        axesBackground, foreground, gridColour);

    rearPathAxes = nexttile(pathLayout, 2);

    plotPathReference(rearPathAxes, ...
        xPosition, yPosition, rearSteering, ...
        segmentStart, segmentEnd, ...
        'Rear steering reference', ...
        'Rear steering [rad]', ...
        axesBackground, foreground, gridColour);

    velocityPathAxes = nexttile(pathLayout, 3);

    plotPathReference(velocityPathAxes, ...
        xPosition, yPosition, velocity, ...
        segmentStart, segmentEnd, ...
        'Velocity reference', ...
        'Velocity [m/s]', ...
        axesBackground, foreground, gridColour);

    accelerationPathAxes = nexttile(pathLayout, 4);

    plotPathReference(accelerationPathAxes, ...
        xPosition, yPosition, acceleration, ...
        segmentStart, segmentEnd, ...
        'Acceleration reference', ...
        'Acceleration [m/s^2]', ...
        axesBackground, foreground, gridColour);

    %% Print diagnostic information

    fprintf('\nComplete controller-reference trajectory\n');
    fprintf('Available snapshots: %d\n', numberOfSnapshots);
    fprintf('Selected snapshots : %s\n', snapshotText);
    fprintf('Detected segments  : %d\n', numberOfSegments);
    fprintf('Trajectory points  : %d\n', size(completeReference,1));
    fprintf('Combined path      : %.3f m\n', totalPathLength);

    for segmentIndex = 1:numberOfSegments

        fprintf(['  Segment %d: snapshot %d, %s, ' ...
                 'length %.3f m\n'], ...
            segmentIndex, ...
            segmentSnapshots(segmentIndex), ...
            directionName(segmentDirections(segmentIndex)), ...
            segmentLengths(segmentIndex));
    end

    fprintf('Front steering     : %+0.4f to %+0.4f rad\n', ...
        min(frontSteering), max(frontSteering));

    fprintf('Rear steering      : %+0.4f to %+0.4f rad\n', ...
        min(rearSteering), max(rearSteering));

    fprintf('Velocity           : %+0.4f to %+0.4f m/s\n', ...
        min(velocity), max(velocity));

    fprintf('Acceleration       : %+0.4f to %+0.4f m/s^2\n\n', ...
        min(acceleration), max(acceleration));

    drawnow;
end


function data = unwrapReferenceLog(data)
% Extract numerical data from common Simulink logging containers.

    if isa(data, 'Simulink.SimulationData.Signal')
        data = data.Values;
    end

    if isa(data, 'timeseries')
        data = data.Data;
    end

    if isstruct(data) && isscalar(data)

        if isfield(data, 'signals') && ...
                isfield(data.signals, 'values')

            data = data.signals.values;

        elseif isfield(data, 'Data')

            data = data.Data;
        end
    end
end


function [selectedSnapshots, selectedDirections] = ...
    findSegmentSnapshots(logData)
% Detect trajectory segments using both path geometry and velocity sign.
% The longest snapshot belonging to each detected segment is retained.

    numberOfSnapshots = size(logData,3);

    selectedSnapshots  = [];
    selectedDirections = [];
    bestSegmentLength  = [];

    representativeSnapshots = {};

    currentSegment = 0;

    for snapshotIndex = 1:numberOfSnapshots

        currentSnapshot = cleanSnapshot( ...
            logData(:,:,snapshotIndex));

        if size(currentSnapshot,1) < 2
            continue;
        end

        currentLength = sum(hypot( ...
            diff(currentSnapshot(:,1)), ...
            diff(currentSnapshot(:,2))));

        finiteValues = currentSnapshot( ...
            isfinite(currentSnapshot));

        containsData = ...
            ~isempty(finiteValues) && ...
            any(abs(finiteValues) > 1e-12);

        if ~containsData || ...
                ~isfinite(currentLength) || ...
                currentLength <= 1e-6

            continue;
        end

        currentDirection = ...
            findReferenceDirection(currentSnapshot);

        if currentSegment == 0

            startsNewSegment = true;

        else

            representativeSnapshot = ...
                representativeSnapshots{currentSegment};

            sameGeometry = pathsBelongToSameSegment( ...
                representativeSnapshot, currentSnapshot);

            storedDirection = ...
                selectedDirections(currentSegment);

            directionChanged = ...
                currentDirection ~= 0 && ...
                storedDirection ~= 0 && ...
                currentDirection ~= storedDirection;

            startsNewSegment = ...
                directionChanged || ~sameGeometry;
        end

        if startsNewSegment

            currentSegment = currentSegment + 1;

            selectedSnapshots(currentSegment) = ...
                snapshotIndex; %#ok<AGROW>

            selectedDirections(currentSegment) = ...
                currentDirection; %#ok<AGROW>

            bestSegmentLength(currentSegment) = ...
                currentLength; %#ok<AGROW>

            representativeSnapshots{currentSegment} = ...
                currentSnapshot; %#ok<AGROW>

        else

            % Recover the segment direction if its first snapshot was
            % stationary.
            if selectedDirections(currentSegment) == 0 && ...
                    currentDirection ~= 0

                selectedDirections(currentSegment) = ...
                    currentDirection;
            end

            % Retain the longest available version of this segment.
            if currentLength > bestSegmentLength(currentSegment)

                selectedSnapshots(currentSegment) = ...
                    snapshotIndex;

                bestSegmentLength(currentSegment) = ...
                    currentLength;

                representativeSnapshots{currentSegment} = ...
                    currentSnapshot;

                if currentDirection ~= 0
                    selectedDirections(currentSegment) = ...
                        currentDirection;
                end
            end
        end
    end
end


function sameSegment = pathsBelongToSameSegment(pathA, pathB)
% Determine whether two snapshots describe the same spatial segment.
%
% A shorter receding reference is considered part of the same segment
% when it is mostly contained in the longer reference.

    pointsA = unique(pathA(:,1:2), ...
        'rows', 'stable');

    pointsB = unique(pathB(:,1:2), ...
        'rows', 'stable');

    if size(pointsA,1) < 2 || size(pointsB,1) < 2
        sameSegment = false;
        return;
    end

    spacingA = hypot( ...
        diff(pointsA(:,1)), diff(pointsA(:,2)));

    spacingB = hypot( ...
        diff(pointsB(:,1)), diff(pointsB(:,2)));

    spacing = [spacingA; spacingB];

    spacing = spacing( ...
        isfinite(spacing) & spacing > 1e-9);

    if isempty(spacing)
        distanceTolerance = 0.05;
    else
        distanceTolerance = max( ...
            0.05, 2.5 * median(spacing));
    end

    % Pairwise squared distances. This does not require an additional
    % MATLAB toolbox.
    deltaX = pointsA(:,1) - pointsB(:,1).';
    deltaY = pointsA(:,2) - pointsB(:,2).';

    distanceSquared = deltaX.^2 + deltaY.^2;
    toleranceSquared = distanceTolerance^2;

    fractionAInsideB = mean( ...
        min(distanceSquared, [], 2) <= toleranceSquared);

    fractionBInsideA = mean( ...
        min(distanceSquared, [], 1) <= toleranceSquared);

    % Using the maximum allows a shortened path to be treated as a
    % subset of the complete path.
    overlapFraction = max( ...
        fractionAInsideB, fractionBInsideA);

    sameSegment = overlapFraction >= 0.60;
end


function snapshot = cleanSnapshot(snapshot)
% Remove rows containing NaN or Inf values.

    validRows = all(isfinite(snapshot), 2);
    snapshot = snapshot(validRows,:);
end


function direction = findReferenceDirection(snapshot)
% Determine trajectory direction from nonzero velocity references.

    velocityTolerance = 1e-5;

    velocity = snapshot(:,5);

    movingVelocity = velocity( ...
        isfinite(velocity) & ...
        abs(velocity) > velocityTolerance);

    if isempty(movingVelocity)
        direction = 0;
        return;
    end

    representativeVelocity = median(movingVelocity);

    if abs(representativeVelocity) <= velocityTolerance
        representativeVelocity = sum(sign(movingVelocity));
    end

    direction = sign(representativeVelocity);
end


function tolerance = calculateJoiningTolerance( ...
    previousReference, currentReference)
% Calculate the tolerance for removing duplicated junction points.

    previousSpacing = hypot( ...
        diff(previousReference(:,1)), ...
        diff(previousReference(:,2)));

    currentSpacing = hypot( ...
        diff(currentReference(:,1)), ...
        diff(currentReference(:,2)));

    spacing = [previousSpacing; currentSpacing];

    spacing = spacing( ...
        isfinite(spacing) & spacing > 1e-9);

    if isempty(spacing)
        tolerance = 1e-6;
    else
        tolerance = max( ...
            1e-6, 0.35 * median(spacing));
    end
end


function figureHandle = getDockedFigure(tag, name, background)
% Create or reuse a separate docked figure tab.

    figureHandle = findall(groot, ...
        'Type', 'figure', ...
        'Tag', tag);

    if isempty(figureHandle)

        figureHandle = figure( ...
            'Name', name, ...
            'NumberTitle', 'off', ...
            'Tag', tag, ...
            'Color', background, ...
            'WindowStyle', 'docked');

    else

        figureHandle = figureHandle(1);

        set(figureHandle, ...
            'Name', name, ...
            'NumberTitle', 'off', ...
            'Color', background, ...
            'WindowStyle', 'docked');

        clf(figureHandle);
        figure(figureHandle);
    end
end


function plotPathReference(ax, x, y, values, ...
    segmentStart, segmentEnd, plotTitle, colorbarLabel, ...
    axesBackground, foreground, gridColour)
% Plot one reference signal along the complete spatial trajectory.

    hold(ax, 'on');

    plot(ax, x, y, ...
        'Color', [0.32 0.32 0.32], ...
        'LineWidth', 0.9, ...
        'HandleVisibility', 'off');

    scatter(ax, x, y, 24, values, ...
        'filled', ...
        'HandleVisibility', 'off');

    % Circle: beginning of every segment
    scatter(ax, ...
        x(segmentStart), ...
        y(segmentStart), ...
        55, 'o', ...
        'MarkerEdgeColor', foreground, ...
        'MarkerFaceColor', 'none', ...
        'LineWidth', 1.2, ...
        'HandleVisibility', 'off');

    % Square: end of every segment
    scatter(ax, ...
        x(segmentEnd), ...
        y(segmentEnd), ...
        55, 's', ...
        'MarkerEdgeColor', foreground, ...
        'MarkerFaceColor', 'none', ...
        'LineWidth', 1.2, ...
        'HandleVisibility', 'off');

    title(ax, plotTitle);
    xlabel(ax, 'x [m]');
    ylabel(ax, 'y [m]');

    axis(ax, 'equal');
    axis(ax, 'tight');

    colormap(ax, turbo(256));
    setColourLimits(ax, values);

    colourBar = colorbar(ax);
    colourBar.Color = foreground;
    colourBar.Label.String = colorbarLabel;
    colourBar.Label.Color = foreground;

    applyDarkAxesStyle(ax, ...
        axesBackground, foreground, gridColour);
end


function setColourLimits(ax, values)
% Prevent invalid colour limits when a reference signal is constant.

    values = values(isfinite(values));

    if isempty(values)
        return;
    end

    minimumValue = min(values);
    maximumValue = max(values);

    if maximumValue > minimumValue

        clim(ax, [minimumValue, maximumValue]);

    else

        padding = max( ...
            1e-6, 0.05 * max(1, abs(minimumValue)));

        clim(ax, [
            minimumValue - padding
            maximumValue + padding
        ]);
    end
end


function addSegmentBoundaries(ax, segmentStart)
% Mark the beginning of every segment after the first.

    validStart = segmentStart(segmentStart > 0);

    for segmentIndex = 2:numel(validStart)

        xline(ax, validStart(segmentIndex) - 0.5, ':', ...
            'Color', [0.55 0.55 0.55], ...
            'LineWidth', 1.0, ...
            'HandleVisibility', 'off');
    end
end


function applyDarkAxesStyle( ...
    ax, background, foreground, gridColour)

    ax.Color = background;
    ax.XColor = foreground;
    ax.YColor = foreground;
    ax.GridColor = gridColour;
    ax.MinorGridColor = gridColour;
    ax.GridAlpha = 0.55;
    ax.MinorGridAlpha = 0.25;
    ax.Box = 'on';

    grid(ax, 'on');

    ax.Title.Color = foreground;
    ax.XLabel.Color = foreground;
    ax.YLabel.Color = foreground;
end


function applyDarkLegendStyle( ...
    legendHandle, background, foreground)

    legendHandle.Color = background;
    legendHandle.TextColor = foreground;
    legendHandle.EdgeColor = [0.55 0.55 0.55];
end


function text = directionName(direction)

    if direction > 0
        text = 'forward';

    elseif direction < 0
        text = 'reverse';

    else
        text = 'stationary/unknown direction';
    end
end