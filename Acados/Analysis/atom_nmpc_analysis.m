%% ATOM_NMPC_ANALYSIS  NMPC prediction vs. actual vehicle state (row 3, acados)
%
% Run it in the Command Window AFTER a simulation of ATOM_parking_chain:
%
%     atom_nmpc_analysis
%
% WHAT IT DOES
%   1. Takes the logged solver calls (out.acadosLog: the 702 inputs/outputs of
%      every acados call) and the logged actual state (out.logsout, signal
%      'x_now' = [X; Y; psi; vx; delta_f; delta_r; ax] of the vehicle).
%   2. Re-solves every logged call offline with atomSolver. The replay is
%      identical to the simulation as long as the solver was not regenerated
%      in between, and it gives the PREDICTED trajectory of each call:
%      9 states x 16 nodes, node k at time t + 0.24 + 0.12*k s (k = 0..15).
%   3. Compares prediction and actual state at the same instants and draws:
%        Figure 1  solver overview: status and SQP iterations of every call
%        Figure 2  actual front/rear steering and heading over the run, with
%                  the predicted trajectory ("fan") drawn every fanEvery s
%        Figure 3  delay-compensation check: prediction error at node 0
%                  (t + 0.24 s) over the run = pure model error
%        Figure 4  prediction error vs. prediction time (nodes 0..15),
%                  converged calls only (mean and 95th percentile)
%        Figure 5+ one snapshot per entry of snapshotTimes: map with
%                  obstacles, corridors, reference, predicted path and car
%                  outlines, actual path; steering predicted vs. actual
%
% PRECONDITIONS (all three are needed)
%   - logging blocks logCat/logToWs inside Subsystem2/acadosSolve
%     (-> out.acadosLog)
%   - the x_now output (5th output) of parkingNMPC named 'x_now' and logged
%     (-> out.logsout)
%   - atomSolver in the workspace, built by the SAME ATOM_Simulink_init as the
%     simulation: do not run ATOM_Simulink_init between the run and this script
%
% The data of the run are saved to tempdir/nmpc_run.mat. If 'out' is not in
% the workspace, the script uses that file instead.
%
% RESULT: struct nmpcAnalysis in the workspace (all arrays behind the figures).
% All the work is done in local functions, so no other workspace variable is
% created or overwritten.

%% Settings
snapshotTimes = [20 42.80];   % [s] moments for the snapshot figures (nearest solver call)
fanEvery      = 0.5;          % [s] draw one predicted trajectory every fanEvery seconds
runFile       = fullfile(tempdir, 'nmpc_run.mat');

%% 1. Data of the run (from 'out', or from the saved file)
if exist('out', 'var') && isa(out, 'Simulink.SimulationOutput')
    if exist('costmap_active', 'var')
        nmpcRun = nmpcCollectRun(out, costmap_active.msg);
    else
        nmpcRun = nmpcCollectRun(out, []);
    end
    save(runFile, '-struct', 'nmpcRun');
    fprintf('Run data taken from ''out'' and saved to %s\n', runFile);
elseif isfile(runFile)
    nmpcRun = load(runFile);
    fprintf('Run data loaded from %s\n', runFile);
else
    error('No simulation output ''out'' in the workspace and no saved file %s.', runFile);
end
if ~exist('atomSolver', 'var')
    error(['atomSolver is not in the workspace. Order: ATOM_Simulink_init, ', ...
           'then the simulation, then atom_nmpc_analysis.']);
end

%% 2.-4. Replay, comparison and figures
nmpcAnalysis = nmpcAnalyse(nmpcRun, atomSolver, snapshotTimes, fanEvery);
clear nmpcRun snapshotTimes fanEvery runFile


%==========================================================================
%  LOCAL FUNCTIONS
%==========================================================================

function R = nmpcCollectRun(out, cmsg)
%NMPCCOLLECTRUN  Logged solver calls + logged actual state of one run.
names = who(out);
if ~any(strcmp(names, 'acadosLog'))
    error(['out.acadosLog not found: add the logging blocks logCat/logToWs ', ...
           'in Subsystem2/acadosSolve and run the simulation again.']);
end
L = out.acadosLog;
D = squeeze(L.signals.values);  if size(D,1) ~= 702, D = D.'; end
if ~any(strcmp(names, 'logsout')) || ~any(strcmp(out.logsout.getElementNames, 'x_now'))
    error(['Signal ''x_now'' not found in out.logsout: name the 5th output line ', ...
           'of parkingNMPC ''x_now'', enable its logging, and run the simulation again.']);
end
xs = out.logsout.getElement('x_now').Values;
XA = squeeze(xs.Data);  if size(XA,1) ~= 7, XA = XA.'; end
R.D    = D;               % 702 x calls: x0, p, yref0, yref, yrefe, xinit, uinit, piinit, status, iter
R.tc   = L.time(:).';     % time of each solver call [s]
R.ta   = xs.Time(:).';    % time of each actual-state sample [s]
R.XA   = XA;              % actual state [X; Y; psi; vx; df; dr; ax] x samples
R.cmsg = cmsg;            % costmap message (map background), may be empty
end

%--------------------------------------------------------------------------
function A = nmpcAnalyse(R, S, snapshotTimes, fanEvery)
%NMPCANALYSE  Replay all calls, compare with the actual state, draw the figures.

% ---- layout of one logged column (see callAcados in parkingNMPC) ----
n = [9 272 7 98 5 144 30 135];  e = cumsum(n);  b = e - n + 1;
D  = R.D;  nC = size(D,2);  tc = R.tc(:).';
stLog = D(701,:);  itLog = D(702,:);
[ta, iu] = unique(R.ta(:).');  XA = R.XA(:, iu);

% ---- 2. replay every call: predicted states, references, corridors ----
Xp  = nan(9, 16, nC);     % predicted [X Y psi vx df dr a cF cR] at the 16 nodes
Ref = nan(5, 16, nC);     % reference [X Y psi df dr] at the 16 nodes
P   = nan(17, 16, nC);    % node parameters [a_des; A(:); b; eps]
st  = zeros(1, nC);  it = zeros(1, nC);
fprintf('Re-solving %d logged solver calls ...\n', nC);
for i = 1:nC
    d = D(:,i);  S.reset();
    S.set('constr_x0', d(b(1):e(1)));
    p = reshape(d(b(2):e(2)), 17, 16);  for j = 0:15, S.set('p', p(:,j+1), j); end
    S.set('cost_y_ref', d(b(3):e(3)), 0);
    yr = reshape(d(b(4):e(4)), 7, 14);   for j = 1:14, S.set('cost_y_ref', yr(:,j), j); end
    S.set('cost_y_ref', d(b(5):e(5)), 15);
    S.set('init_x',  reshape(d(b(6):e(6)), 9, 16));
    S.set('init_u',  reshape(d(b(7):e(7)), 2, 15));
    S.set('init_pi', reshape(d(b(8):e(8)), 9, 15));
    S.solve();
    Xp(:,:,i) = S.get('x');
    st(i) = S.get('status');  it(i) = S.get('sqp_iter');
    y0 = d(b(3):e(3));  ye = d(b(5):e(5));
    Ref(:,:,i) = [y0(1:5), yr(1:5,:), ye(1:5)];
    P(:,:,i) = p;
end
nMismatch = sum(st ~= stLog | it ~= itLog);

% ---- 3. actual state at the node times, and the prediction errors ----
th  = 0.24 + 0.12*(0:15);                 % prediction time of each node [s]
tn  = tc(:) + th;                         % absolute node times, calls x 16
XaN = nan(7, 16, nC);
for i = 1:nC
    XaN(:,:,i) = interp1(ta, XA.', tn(i,:)).';
end
posErr = squeeze(hypot(Xp(1,:,:) - XaN(1,:,:), Xp(2,:,:) - XaN(2,:,:)));       % [m]
psiErr = squeeze(atan2(sin(Xp(3,:,:) - XaN(3,:,:)), cos(Xp(3,:,:) - XaN(3,:,:)))) * 180/pi;  % [deg]
dfErr  = squeeze(Xp(5,:,:) - XaN(5,:,:));                                        % [rad]
drErr  = squeeze(Xp(6,:,:) - XaN(6,:,:));                                        % [rad]
if nC == 1   % squeeze keeps 16 x 1 only for several calls
    posErr = posErr(:); psiErr = psiErr(:); dfErr = dfErr(:); drErr = drErr(:);
end

% ---- summary in the Command Window ----
ok = (st == 0);
fprintf('\n=== NMPC prediction analysis ===\n');
fprintf('solver calls %d | status 0: %d | status 2: %d | other: %d | iterations mean %.1f, max %d\n', ...
    nC, sum(st==0), sum(st==2), sum(st~=0 & st~=2), mean(it), max(it));
if nMismatch == 0
    fprintf('replay identical to the simulation in all calls (status and iterations)\n');
else
    warning(['Replay differs from the simulation in %d calls: the solver is probably ', ...
             'not the one used in the run (ATOM_Simulink_init run in between?).'], nMismatch);
end
[m0, q0] = nmpcRowStats(posErr(1,:));  [h0, hq0] = nmpcRowStats(psiErr(1,:));
fprintf('node  0 (t+0.24 s, delay compensation): position error mean %.1f cm, 95%% %.1f cm | heading mean %.2f, 95%% %.2f deg\n', ...
    100*m0, 100*q0, h0, hq0);
[m15, q15] = nmpcRowStats(posErr(16, ok));  [h15, hq15] = nmpcRowStats(psiErr(16, ok));
fprintf('node 15 (t+2.04 s, converged calls):    position error mean %.1f cm, 95%% %.1f cm | heading mean %.2f, 95%% %.2f deg\n', ...
    100*m15, 100*q15, h15, hq15);
if any(~ok)
    fprintf('non-converged calls at t = '); fprintf('%.2f ', tc(~ok)); fprintf('\n');
end

% ---- Figure 1: solver overview ----
figure('Name', 'NMPC 1: solver overview');
subplot(2,1,1);  stairs(tc, st, 'LineWidth', 1.2);  grid on;
ylabel('acados status');  title('Solver status of every call (0 = converged, 2 = max. iterations)');
subplot(2,1,2);  stairs(tc, it, 'LineWidth', 1.2);  grid on;
ylabel('SQP iterations');  xlabel('t [s]');  title('SQP iterations of every call');

% ---- Figure 2: actual signals with prediction fans ----
psiA  = unwrap(XA(3,:));                                  % continuous heading [rad]
k     = floor(tc / fanEvery);  fanIdx = find([true, diff(k) ~= 0]);
labels = {'front steering \delta_f [rad]', 'rear steering \delta_r [rad]', 'heading \psi [deg]'};
figure('Name', 'NMPC 2: predictions vs. actual');
for s = 1:3
    subplot(3,1,s);  hold on;  grid on;
    if s < 3
        rx = 4 + s;                                       % row of df / dr in Xp and XA
        hA = plot(ta, XA(rx,:), 'g-', 'LineWidth', 2);
        [tg, yg] = nmpcWithGaps(tc + 0.24, squeeze(Ref(rx-1,1,:)).');
        hR = plot(tg, yg, 'b--', 'LineWidth', 1);
        for i = fanIdx
            hP = plot(tn(i,:), Xp(rx,:,i), 'm-');
        end
    else
        hA = plot(ta, psiA*180/pi, 'g-', 'LineWidth', 2);
        psiAt0 = interp1(ta, psiA, tc + 0.24);            % actual heading at node 0
        refPsi = squeeze(Ref(3,1,:)).';
        refPsi = refPsi + 2*pi*round((psiAt0 - refPsi)/(2*pi));
        [tg, yg] = nmpcWithGaps(tc + 0.24, refPsi*180/pi);
        hR = plot(tg, yg, 'b--', 'LineWidth', 1);
        for i = fanIdx
            y = Xp(3,:,i);
            if ~isnan(psiAt0(i)), y = y + 2*pi*round((psiAt0(i) - y(1))/(2*pi)); end
            hP = plot(tn(i,:), y*180/pi, 'm-');
        end
    end
    yl = ylim;
    if any(~ok)
        hF = plot(tc(~ok), yl(1)*ones(1, sum(~ok)), 'r.', 'MarkerSize', 10);
        legend([hA hR hP hF], {'actual', 'reference (node 0)', ...
            sprintf('predicted (every %.1f s)', fanEvery), 'non-converged call'}, 'Location', 'best');
    else
        legend([hA hR hP], {'actual', 'reference (node 0)', ...
            sprintf('predicted (every %.1f s)', fanEvery)}, 'Location', 'best');
    end
    title(labels{s});
end
xlabel('t [s]');

% ---- Figure 3: delay compensation (node 0) over the run ----
t0 = tn(:,1).';
figure('Name', 'NMPC 3: delay compensation (node 0)');
subplot(3,1,1);  [tg, yg] = nmpcWithGaps(t0, 100*posErr(1,:));  plot(tg, yg, 'LineWidth', 1.2);  grid on;
ylabel('position [cm]');
title('Error at node 0 (t + 0.24 s) = pure model error');
subplot(3,1,2);  [tg, yg] = nmpcWithGaps(t0, psiErr(1,:));  plot(tg, yg, 'LineWidth', 1.2);  grid on;
ylabel('heading [deg]');
subplot(3,1,3);  hold on;  grid on;
[tg, yg] = nmpcWithGaps(t0, dfErr(1,:));  plot(tg, yg, 'LineWidth', 1.2);
[tg, yg] = nmpcWithGaps(t0, drErr(1,:));  plot(tg, yg, 'LineWidth', 1.2);
ylabel('steering [rad]');  xlabel('t [s]');  legend('front \delta_f', 'rear \delta_r', 'Location', 'best');

% ---- Figure 4: prediction error vs. prediction time ----
figure('Name', 'NMPC 4: prediction error vs. prediction time');
E    = {100*posErr(:,ok), psiErr(:,ok), dfErr(:,ok), drErr(:,ok)};
ylab = {'position [cm]', 'heading [deg]', 'front steering [rad]', 'rear steering [rad]'};
for s = 1:4
    [mE, qE] = nmpcRowStats(E{s});
    subplot(2,2,s);  plot(th, mE, 'o-', th, qE, 's--', 'LineWidth', 1.2);  grid on;
    xlabel('prediction time [s]');  ylabel(['|error| ' ylab{s}]);
    legend('mean', '95th percentile', 'Location', 'northwest');
end
sgtitle(sprintf('Predicted minus actual, %d converged calls (node 0 = 0.24 s ... node 15 = 2.04 s)', sum(ok)));

% ---- Figure 5+: snapshots ----
for tq = snapshotTimes
    [~, i] = min(abs(tc - tq));
    nmpcSnapshot(i, tc, tn, Xp, XaN, Ref, P, ta, XA, st, it, posErr, psiErr, R.cmsg);
end

% ---- result ----
A.tc = tc;  A.nodeTime = th;  A.tn = tn;
A.status = st;  A.iter = it;  A.statusLog = stLog;  A.iterLog = itLog;
A.Xpred = Xp;  A.Xactual = XaN;  A.Ref = Ref;  A.P = P;
A.posErr = posErr;  A.psiErr = psiErr;  A.dfErr = dfErr;  A.drErr = drErr;
A.description = ['Xpred 9x16xcalls [X Y psi vx df dr a cF cR]; Xactual 7x16xcalls ', ...
                 '[X Y psi vx df dr ax] at the node times; errors 16 x calls (predicted - actual)'];
end

%--------------------------------------------------------------------------
function nmpcSnapshot(i, tc, tn, Xp, XaN, Ref, P, ta, XA, st, it, posErr, psiErr, cmsg)
%NMPCSNAPSHOT  Map + steering for one solver call.
fprintf('snapshot t = %.2f s | status %d, %d iterations | node 0: %.1f cm, %.2f deg | node 15: %.1f cm, %.2f deg\n', ...
    tc(i), st(i), it(i), 100*posErr(1,i), psiErr(1,i), 100*posErr(16,i), psiErr(16,i));
figure('Name', sprintf('NMPC snapshot t = %.2f s', tc(i)));
subplot(2,2,[1 3]);  hold on;  grid on;
if ~isempty(cmsg)
    m = cmsg.info;  res = double(m.resolution);  W = double(m.width);  H = double(m.height);
    G = double(cmsg.data);  if isvector(G), G = reshape(G, W, H).'; end
    imagesc(double(m.origin.position.x) + (0:W-1)*res, double(m.origin.position.y) + (0:H-1)*res, ...
        double(G >= 100 | G < 0));
    colormap(gca, [1 1 1; 0.85 0.55 0.55]);  set(gca, 'YDir', 'normal');
end
axis equal;
pr = [1 3; 1 4; 2 4; 2 3];                % corner = intersection of two faces
for k = [1 6 11 16]                       % corridors of nodes 0, 5, 10, 15
    Ak = reshape(P(2:9,k,i), 4, 2);  bk = P(10:13,k,i);  C = zeros(2,4);
    for c = 1:4, C(:,c) = Ak(pr(c,:),:) \ bk(pr(c,:)); end
    hC = plot(C(1,[1:4 1]), C(2,[1:4 1]), ':', 'Color', [0.4 0.4 0.4]);
end
hR = plot(Ref(1,:,i), Ref(2,:,i), 'b--', 'LineWidth', 1);
hP = plot(Xp(1,:,i), Xp(2,:,i), 'm.-', 'LineWidth', 1.5, 'MarkerSize', 12);
w  = ta >= tc(i) & ta <= tn(i,end);
hA = plot(XA(1,w), XA(2,w), 'g-', 'LineWidth', 2);
plot([Xp(1,:,i); XaN(1,:,i)], [Xp(2,:,i); XaN(2,:,i)], 'k-');       % error per node
body = [2.71 2.71 -0.71 -0.71 2.71; 1.328 -1.328 -1.328 1.328 1.328]; % expanded footprint
for k = [1 16]
    c = cos(Xp(3,k,i));  s = sin(Xp(3,k,i));
    q = [c -s; s c]*body + Xp(1:2,k,i);
    plot(q(1,:), q(2,:), 'm-');
end
xy = [Xp(1:2,:,i), XaN(1:2,~isnan(XaN(1,:,i)),i), Ref(1:2,:,i)];
xlim([min(xy(1,:))-3, max(xy(1,:))+3]);  ylim([min(xy(2,:))-3, max(xy(2,:))+3]);
legend([hR hP hA hC], {'reference', 'NMPC prediction', 'actual (afterwards)', ...
    'corridors (nodes 0,5,10,15)'}, 'Location', 'best');
title(sprintf('t = %.2f s, status %d, %d iterations', tc(i), st(i), it(i)));
xlabel('X [m]');  ylabel('Y [m]');
names = {'front steering \delta_f [rad]', 'rear steering \delta_r [rad]'};
for s = 1:2
    subplot(2,2,2*s);  hold on;  grid on;
    plot(tn(i,:), Xp(4+s,:,i), 'm.-', tn(i,:), XaN(4+s,:,i), 'g-', tn(i,:), Ref(3+s,:,i), 'b--', ...
        'LineWidth', 1.2);
    title(names{s});  xlabel('t [s]');
    legend('predicted', 'actual', 'reference', 'Location', 'best');
end
end

%--------------------------------------------------------------------------
function [m, q] = nmpcRowStats(E)
%NMPCROWSTATS  Mean and 95th percentile of |E| per row, ignoring NaN.
m = nan(size(E,1), 1);  q = m;
for r = 1:size(E,1)
    v = sort(abs(E(r, ~isnan(E(r,:)))));
    if isempty(v), continue; end
    m(r) = mean(v);  q(r) = v(max(1, ceil(0.95*numel(v))));
end
end

%--------------------------------------------------------------------------
function [tg, yg] = nmpcWithGaps(t, y)
%NMPCWITHGAPS  Insert NaN where calls are missing (wheel alignment), so that
%   plot() does not draw a line across the gap.
t = t(:).';  y = y(:).';
gap = find(diff(t) > 0.06);
tg = t;  yg = y;
for g = fliplr(gap)
    tg = [tg(1:g), NaN, tg(g+1:end)];
    yg = [yg(1:g), NaN, yg(g+1:end)];
end
end