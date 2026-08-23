clc
clear all
close all

L = 2;                      % wheelbase [m]
delta_max = 0.7;

rr = linspace(-1,1,101);    % rear-steer ratio

curvature = zeros(size(rr));
R = zeros(size(rr));        % radius of curvature
gain_4WS = zeros(size(rr)); % required gain


%%=============================================
%
% Figure 1,2,4
%
%==============================================

% %% Curvature and radius with fixed delta_f
% for i = 1:length(rr)
% 
%     curvature(i) = sin((1-rr(i))*delta_max)/(L*cos(rr(i)*delta_max));
%     R(i) = 1/curvature(i);
% 
% end
% 
% figure(1)
% plot(rr, R, 'b', 'LineWidth',2)
% hold on
% grid on
% 
% % Evidenzia rr = 0
% idx0 = find(rr==0);
% 
% plot(rr(idx0), R(idx0), 'ro', ...
%     'MarkerSize',10, ...
%     'MarkerFaceColor','r')
% 
% text(rr(idx0), R(idx0), ...
%     sprintf('  rr = 0\n  R = %.2f m', R(idx0)), ...
%     'FontSize',10)
% 
% % Evidenzia minimo R
% [Rmin, idxMin] = min(R);
% 
% plot(rr(idxMin), Rmin, 'ks', ...
%     'MarkerSize',10, ...
%     'MarkerFaceColor','g')
% 
% text(rr(idxMin), Rmin, ...
%     sprintf('  R_{min}=%.2f m\n  rr=%.2f', Rmin, rr(idxMin)), ...
%     'FontSize',10)
% 
% xlabel('Rear-steer ratio, rr')
% ylabel('Radius of curvature, R [m]')
% title('Effect of rear steering ratio on turning radius (gain4WS = 1, delta_f(max) = 0.7 rad)')
% legend('R(rr)','2WS case (rr=0)','Minimum radius')
% 
% 
% 
% %% Figure 2 and 4
% N = 10;                                 % N° of reference radii of curvature
% 
% Rmin_2WS = L/sin(delta_max);
% Rmax = 10;
% 
% delta_2WS_vec = zeros(1,N);
% delta_2WS_low_vec = zeros(1,N);
% 
% rr = zeros(101, N);           % rr: 101 righe (campioni) x N colonne (R_ref)
% gain4WS = zeros(101, N);      % gain4WS: stessa dimensione di rr
% gain4WS_low = zeros(101,N);
% 
% R_ref = linspace(Rmin_2WS, Rmax, N);
% R_ref_low = linspace(Rmin,Rmin_2WS,N);
% 
% 
% 
% %% Rear steering ratio range
% 
% for j = 1:N    
%     delta_2WS = min(asin(L/R_ref(j)), delta_max);
%     delta_2WS_low = delta_max;
% 
%     % Memorizza lo sterzo equivalente 2WS
%     delta_2WS_vec(j) = delta_2WS; 
%     delta_2WS_low_vec(j) = delta_2WS_low;
% 
%     R_low = R_ref_low(j);
% 
%     rr(:,j) = linspace(-1,1,101)';
% 
%     % Solve gain4WS for each rr
%     for i = 1:length(rr(:,j))
%         % Equation:
%         % tan(delta_2WS) =
%         % sin((1-rr)*gain*delta_2WS)/cos(rr*gain*delta_2WS)
% 
%         eq1 = @(gain) ...
%             sin((1-rr(i,j))*gain*delta_2WS) / ...
%             cos(rr(i,j)*gain*delta_2WS) ...
%             - sin(delta_2WS);
% 
%         eq2 = @(gain) ...
%             sin((1-rr(i,j))*gain*delta_max) ...
%             /(L*cos(rr(i,j)*gain*delta_max)) ...
%             - 1/R_low;
% 
%         v01 = eq1(0);
%         v11 = eq1(1.5);
%         if v01 * v11 < 0
%             gain4WS(i,j) = fzero(eq1, [0 1.5]);
%         else
%             % Nessuna radice nell'intervallo [0,1.5]: usare valore di fallback o NaN
%             gain4WS(i,j) = NaN;
%         end
% 
% 
%         v02 = eq2(0);
%         v12 = eq2(1.5);
%         if v02 * v12 < 0
%             gain4WS_low(i,j) = fzero(eq2, [0 1.5]);
%         else
%             % Nessuna radice nell'intervallo [0,1.5]: usare valore di fallback o NaN
%             gain4WS_low(i,j) = NaN;
%         end
% 
%     end
% end
% 
% %% Plot
% 
% figure(2)
% clf
% hold on
% grid on
% 
% cmap = turbo(N);
% 
% for j = 1:N
%     plot(rr(:,j), gain4WS(:,j), ...
%         'Color', cmap(j,:), ...
%         'LineWidth',2);
% end
% 
% xlabel('Rear steering ratio, rr')
% ylabel('gain_{4WS}')
% 
% xlim([-1 0])
% ylim([0 1.5])
% 
% %% Colorbar
% 
% colormap(cmap)
% 
% cb = colorbar;
% 
% caxis([1 N])
% 
% cb.Ticks = 1:N;
% 
% labels = strings(1,N);
% 
% for j = 1:N
% 
%     labels(j) = sprintf('\\delta_{2WS}=%.2f rad, R_{ref}=%.2f m', ...
%         delta_2WS_vec(j), R_ref(j));
% 
% end
% 
% cb.TickLabels = labels;
% 
% title({
%     'Relation between rr and gain_{4WS}'
%     sprintf('(valid up to \\delta_{2WS,max}=%.2f rad, R_{min}=%.2f m)', ...
%     delta_max, Rmin_2WS)
%     })
% 
% 
% %% ============================================================
% % Figure 4
% % gain4WS(rr, delta2WS)
% % ============================================================
% 
% figure(4)
% clf
% 
% % Mesh della superfice
% [RR, RADIUS] = meshgrid(rr(:,1), R_ref);
% 
% [RR, RADIUS_LOW] = meshgrid(rr(:,1), R_ref_low);
% 
% % gain4WS ha dimensioni:
% %   101 x N
% % mentre meshgrid restituisce:
% %   N x 101
% % quindi trasponiamo gain4WS
% 
% surf(RR, RADIUS, gain4WS', ...
%     'EdgeColor','none', ...
%     'FaceAlpha',0.95);
% hold on
% surf(RR,RADIUS_LOW,gain4WS_low',  ...
%     'EdgeColor','none', ...
%     'FaceAlpha',0.95);
% grid on
% box on
% 
% % Insert lines of figure 2
% % cmap = turbo(N);
% % 
% % for j = 1:N
% % 
% %     valid = ~isnan(gain4WS(:,j));
% % 
% %     plot3( ...
% %         rr(valid,j), ...
% %         R_ref(j)*ones(sum(valid),1), ...
% %         gain4WS(valid,j), ...
% %         'Color',cmap(j,:), ...
% %         'LineWidth',2);
% % 
% % end
% 
% xlabel('Rear steering ratio, rr')
% ylabel('Reference turning radius, R_{ref} [m]')
% zlabel('gain_{4WS}')
% 
% title({
%     'Front steering reduction gain'
%     sprintf('(valid up to \\delta_{2WS,max}=%.2f rad)',delta_max)
%     })
% 
% view(40,30)
% 
% xlim([-1 0])
% ylim([R_ref_low(1) R_ref(end)])
% zlim([0 1])
% 
% colormap(turbo)
% 
% cb = colorbar;
% cb.Label.String = 'gain_{4WS}';
% 
% clim([0.5 1])






%% ============================================================
% Figure 5
% gain4WS(rr, curvature)
%
% Exact 4WS kinematic model:
%
% kappa = sin((1-rr)*delta_f) / (L*cos(rr*delta_f))
%
% delta_f = gain4WS * delta_2WS
%
% Region 1:
%   delta_2WS = asin(L*kappa)
%
% Region 2:
%   delta_2WS = delta_max
%
% ============================================================

clc
clear

% ============================================================
% Vehicle parameters
% ============================================================

L = 2;                  % Wheelbase [m]

delta_max = 0.7;        % Maximum Stanley steering angle [rad]


% ============================================================
% Rear steering ratio
% ============================================================

N_rr = 1001;

rr_vec = linspace(-1,0,N_rr);


% ============================================================
% Maximum curvature
% ============================================================

% 2WS maximum curvature
%
% rr = 0
% delta_f = delta_max

kappa_2WS_max = ...
    sin(delta_max) / ...
    (L*cos(0*delta_max));


% Absolute 4WS maximum curvature
%
% rr = -1
% delta_f = delta_max

kappa_4WS_max = ...
    sin(2*delta_max) / ...
    (L*cos(-1*delta_max));


% ============================================================
% Curvature ranges
% ============================================================

N_kappa_high = 1001;
N_kappa_low  = 1001;


% Region 1:
% 0 <= kappa <= kappa_2WS_max

kappa_ref = ...
    linspace(0,kappa_2WS_max,N_kappa_high);


% Region 2:
% kappa_2WS_max <= kappa <= kappa_4WS_max

kappa_ref_low = ...
    linspace(kappa_2WS_max,kappa_4WS_max,N_kappa_low);


% ============================================================
% Allocate gain matrices
% ============================================================

gain4WS = NaN(N_rr,N_kappa_high);

gain4WS_low = NaN(N_rr,N_kappa_low);


% ============================================================
% Numerical tolerance
% ============================================================

tol = 1e-10;

% ============================================================
% REGION 1
%
% Stanley is NOT saturated
%
% delta_2WS = asin(L*kappa)
%
% If the requested curvature is not reachable for a given rr,
% gain4WS is saturated to 1.
% ============================================================

for j = 1:length(kappa_ref)

    kappa = kappa_ref(j);

    % Equivalent 2WS steering angle
    delta_2WS = asin(L*kappa);


    for i = 1:length(rr_vec)

        rr_current = rr_vec(i);


        % ====================================================
        % Zero curvature
        % ====================================================

        if abs(kappa) < tol

            gain4WS(i,j) = 1;

            continue

        end


        % ====================================================
        % Maximum curvature achievable for this rr
        % with gain = 1
        % ====================================================

        kappa_max_rr = ...
            sin((1-rr_current)*delta_2WS) / ...
            (L*cos(rr_current*delta_2WS));


        % ====================================================
        % Requested curvature is reachable
        % ====================================================

        if kappa <= kappa_max_rr + tol


            % If exactly at the maximum, gain = 1

            if abs(kappa-kappa_max_rr) < tol

                gain4WS(i,j) = 1;

                continue

            end


            % =================================================
            % Solve for gain in [0,1]
            % =================================================

            eq = @(gain) ...
                sin((1-rr_current)*gain*delta_2WS) / ...
                (L*cos(rr_current*gain*delta_2WS)) ...
                - kappa;


            gain4WS(i,j) = ...
                fzero(eq,[0 1]);


        % ====================================================
        % Requested curvature is NOT reachable
        %
        % Saturate gain to 1
        % ====================================================

        else

            gain4WS(i,j) = 1;

        end

    end

end



% ============================================================
% REGION 2
%
% Stanley saturated:
%
% delta_2WS = delta_max
%
% If the requested curvature is not reachable for a given rr,
% gain4WS is saturated to 1.
% ============================================================

for j = 1:length(kappa_ref_low)

    kappa = kappa_ref_low(j);

    % Stanley saturated
    delta_2WS = delta_max;


    for i = 1:length(rr_vec)

        rr_current = rr_vec(i);


        % ====================================================
        % Maximum curvature achievable for this rr
        % with gain = 1
        % ====================================================

        kappa_max_rr = ...
            sin((1-rr_current)*delta_2WS) / ...
            (L*cos(rr_current*delta_2WS));


        % ====================================================
        % Requested curvature is reachable
        % ====================================================

        if kappa <= kappa_max_rr + tol


            % Exactly at maximum curvature

            if abs(kappa-kappa_max_rr) < tol

                gain4WS_low(i,j) = 1;

                continue

            end


            % =================================================
            % Solve for gain in [0,1]
            % =================================================

            eq = @(gain) ...
                sin((1-rr_current)*gain*delta_2WS) / ...
                (L*cos(rr_current*gain*delta_2WS)) ...
                - kappa;


            gain4WS_low(i,j) = ...
                fzero(eq,[0 1]);


        % ====================================================
        % Requested curvature is NOT reachable
        %
        % Saturate gain to 1
        % ====================================================

        else

            gain4WS_low(i,j) = 1;

        end

    end

end


% ============================================================
% Meshes
% ============================================================

[RR_high,KAPPA_high] = ...
    meshgrid(rr_vec,kappa_ref);

[RR_low,KAPPA_low] = ...
    meshgrid(rr_vec,kappa_ref_low);


% ============================================================
% FIGURE
% ============================================================

figure(5)
clf

hold on
grid on
box on


% Region 1

surf( ...
    RR_high, ...
    KAPPA_high, ...
    gain4WS', ...
    'EdgeColor','none', ...
    'FaceAlpha',0.95);


% Region 2

surf( ...
    RR_low, ...
    KAPPA_low, ...
    gain4WS_low', ...
    'EdgeColor','none', ...
    'FaceAlpha',0.95);


% ============================================================
% 2WS saturation boundary
% ============================================================

plot3( ...
    [-1 0], ...
    [kappa_2WS_max kappa_2WS_max], ...
    [1 1], ...
    'k--', ...
    'LineWidth',2);


% ============================================================
% Labels
% ============================================================

xlabel('Rear steering ratio, rr')

ylabel('Reference curvature, \kappa_{ref} [1/m]')

zlabel('gain_{4WS}')


title({
    'Front steering reduction gain'
    sprintf('\\delta_{2WS,max}=%.2f rad, \\kappa_{2WS,max}=%.3f 1/m', ...
    delta_max,kappa_2WS_max)
    })


% ============================================================
% View
% ============================================================

view(40,30)

xlim([-1 0])

ylim([0 kappa_4WS_max])

zlim([0 1])


% ============================================================
% Colormap
% ============================================================

colormap(turbo)

cb = colorbar;

cb.Label.String = 'gain_{4WS}';

clim([0.5 1]);



% ============================================================
% Save LUT 2
% gain4WS = f(rr, kappa_ref)
% ============================================================

% ============================================================
% Combine Region 1 and Region 2
% gain4WS = f(rr, kappa_ref)
% ============================================================

% Remove duplicated boundary point
% kappa_2WS_max appears in both regions

kappa_gain_BP = [ ...
    kappa_ref, ...
    kappa_ref_low(2:end) ...
    ];

gain4WS_LUT = [ ...
    gain4WS, ...
    gain4WS_low(:,2:end) ...
    ];

% Rear steering ratio breakpoint
rr_gain_BP = rr_vec;


% ============================================================
% Check dimensions
% ============================================================

disp('LUT dimensions:')

disp(['rr_gain_BP:    ', mat2str(size(rr_gain_BP))])
disp(['kappa_gain_BP: ', mat2str(size(kappa_gain_BP))])
disp(['gain4WS_LUT:   ', mat2str(size(gain4WS_LUT))])


% ============================================================
% Save LUT
% ============================================================

save('/home/andrea-ricetti/Documenti/MATLAB/LUT/LUT_gain4WS_computation.mat', ...
    'rr_gain_BP', ...
    'kappa_gain_BP', ...
    'gain4WS_LUT');





%%================================
% Figure 3
%
%=================================

% Rsurf = repmat(R_ref, length(rr(:,1)), 1);   % 101 x N
% 
% figure(3)
% clf
% 
% Rsurf = repmat(R_ref,length(rr(:,1)),1);
% 
% surf(rr, gain4WS, Rsurf,...
%     'EdgeColor','none',...
%     'FaceAlpha',0.9)
% 
% hold on
% grid on
% box on
% 
% xlabel('Rear steering ratio, rr')
% ylabel('gain_{4WS}')
% zlabel('Reference turning radius, R_{ref} [m]')
% 
% title({
%     'Reference turning radius'
%     sprintf('(valid up to \\delta_{2WS,max}=%.2f rad)',delta_max)
%     })
% 
% view(40,30)
% 
% xlim([-1 0])
% ylim([0 1.2])
% zlim([R_ref(1) R_ref(end)])
% 
% colormap(turbo)
% 
% cb = colorbar;
% cb.Label.String = 'Reference turning radius [m]';
% 
% clim([R_ref(1) R_ref(end)])


%% ============================================================
% Figure 6
%
% Rear steering ratio as a function of:
%   - velocity
%   - reference turning radius
%
% Exact 4WS kinematic model:
%
% R = L*cos(rr*delta_f) / sin((1-rr)*delta_f)
%
% ============================================================

% clc
% 
% 
% %% ============================================================
% % Vehicle parameters
% % ============================================================
% 
% L = 2;                  % Wheelbase [m]
% 
% delta_max = 0.7;        % Maximum front steering angle [rad]
% 
% 
% %% ============================================================
% % Rear steering law parameters
% % ============================================================
% 
% kappa0 = 0.1;           % Curvature sensitivity [1/m]
% 
% v0 = 15/3.6;            % Transition velocity [m/s]
% 
% kv_min = 0.2;           % Minimum velocity factor
% 
% sv = 3/3.6;             % Velocity transition width [m/s]
% 
% 
% %% ============================================================
% % Parameter ranges
% % ============================================================
% 
% N_v  = 1001;
% N_R  = 1001;
% N_rr = 1001;
% 
% velocity_vec = linspace(0,30/3.6,N_v);   % [m/s]
% 
% R_vec = linspace(0.5,10,N_R);            % [m]
% 
% rr_vec = linspace(-1,0,N_rr);
% 
% 
% [V,R] = meshgrid(velocity_vec,R_vec);
% 
% 
% %% ============================================================
% % Velocity contribution
% %
% % kv -> 1       for v << v0
% % kv -> kv_min  for v >> v0
% % ============================================================
% 
% kv = kv_min + ...
%      (1-kv_min) ./ ...
%      (1 + exp((V-v0)/sv));
% 
% 
% %% ============================================================
% % Rear steering ratio law
% %
% % rr = f(R,v)
% % ============================================================
% 
% RR = -(1-exp(-1./(R*kappa0))) .* kv;
% 
% 
% %% ============================================================
% % Saturation
% %
% % rr is restricted to [-1,0]
% % ============================================================
% 
% RR = max(min(RR,0),-1);
% 
% 
% %% ============================================================
% % Kinematic minimum radius constraint
% %
% % At maximum front steering:
% %
% % delta_f = delta_max
% %
% % Exact curvature:
% %
% % kappa =
% % sin((1-rr)*delta_f) / (L*cos(rr*delta_f))
% %
% % Therefore:
% %
% % R_min(rr) =
% % L*cos(rr*delta_max) /
% % sin((1-rr)*delta_max)
% %
% % ============================================================
% 
% R_min_rr = L*cos(rr_vec*delta_max) ./ ...
%            sin((1-rr_vec)*delta_max);
% 
% 
% %% ============================================================
% % Remove physically unreachable combinations
% %
% % For a given rr:
% %
% % R < R_min(rr)  --> unreachable
% %
% % ============================================================
% 
% for j = 1:N_R
% 
%     for i = 1:N_v
% 
%         rr_current = RR(j,i);
% 
%         Rmin_current = ...
%             L*cos(rr_current*delta_max) / ...
%             sin((1-rr_current)*delta_max);
% 
%         if R(j,i) < Rmin_current
%             RR(j,i) = NaN;
%         end
% 
%     end
% 
% end
% 
% 
% %% ============================================================
% % Important reference radii
% % ============================================================
% 
% % 2WS minimum radius (rr = 0)
% 
% Rmin_2WS = L*cos(0*delta_max) / ...
%            sin(delta_max);
% 
% 
% % Absolute minimum radius with rr = -1
% 
% Rmin_4WS = L*cos(-delta_max) / ...
%            sin(2*delta_max);
% 
% 
% %% ============================================================
% % Surface
% % ============================================================
% 
% figure(6)
% clf
% 
% surf(V*3.6,R,RR,...
%     'EdgeColor','none',...
%     'FaceAlpha',0.95)
% 
% hold on
% grid on
% box on
% 
% 
% %% ============================================================
% % Kinematic constraint curve
% %
% % R_min as a function of rr
% % ============================================================
% 
% plot3( ...
%     zeros(size(rr_vec)), ...
%     R_min_rr, ...
%     rr_vec, ...
%     'r', ...
%     'LineWidth',3);
% 
% 
% %% ============================================================
% % 2WS minimum-radius line
% % ============================================================
% 
% plot3( ...
%     [0 30], ...
%     [Rmin_2WS Rmin_2WS], ...
%     [0 0], ...
%     'k--', ...
%     'LineWidth',2);
% 
% 
% %% ============================================================
% % Absolute 4WS minimum-radius line
% % ============================================================
% 
% plot3( ...
%     [0 30], ...
%     [Rmin_4WS Rmin_4WS], ...
%     [-1 -1], ...
%     'm--', ...
%     'LineWidth',2);
% 
% 
% %% ============================================================
% % Labels
% % ============================================================
% 
% xlabel('Velocity [km/h]')
% 
% ylabel('Reference turning radius, R_{ref} [m]')
% 
% zlabel('Rear steering ratio, rr')
% 
% 
% title({
%     'Rear steering ratio as a function of velocity and radius'
%     sprintf('\\delta_{f,max}=%.2f rad, rr \\in [-1,0]',delta_max)
%     })
% 
% 
% %% ============================================================
% % Axis limits
% % ============================================================
% 
% view(40,30)
% 
% xlim([0 30])
% 
% ylim([0.5 10])
% 
% zlim([-1 0])
% 
% 
% %% ============================================================
% % Colormap
% % ============================================================
% 
% colormap(turbo)
% 
% cb = colorbar;
% 
% cb.Label.String = 'Rear steering ratio, rr';
% 
% clim([-1 0])




%% ============================================================
% Figure 7
%
% Rear steering ratio as a function of:
%   - velocity
%   - reference curvature
%
% Exact 4WS kinematic model:
%
% kappa =
% sin((1-rr)*delta_f)
% -------------------
% L*cos(rr*delta_f)
%
% ============================================================

% clc
% 
% 
% %% ============================================================
% % Vehicle parameters
% % ============================================================
% 
% L = 2;                  % Wheelbase [m]
% 
% delta_max = 0.7;        % Maximum front steering angle [rad]
% 
% 
% %% ============================================================
% % Rear steering law parameters
% % ============================================================
% 
% kappa0 = 0.1;           % Curvature sensitivity [1/m]
% 
% v0 = 15/3.6;            % Transition velocity [m/s]
% 
% kv_min = 0.2;           % Minimum velocity factor
% 
% sv = 1/3.6;             % Velocity transition width [m/s]
% 
% 
% %% ============================================================
% % Parameter ranges
% % ============================================================
% 
% N_v     = 1001;
% N_kappa = 1001;
% 
% velocity_vec = linspace(0,30/3.6,N_v);       % [m/s]
% 
% % Maximum theoretically reachable curvature:
% %
% % rr = -1
% % delta_f = delta_max
% 
% kappa_max_4WS = ...
%     sin(2*delta_max) / ...
%     (L*cos(-delta_max));
% 
% 
% kappa_vec = linspace(0,kappa_max_4WS,N_kappa);
% 
% 
% [V,KAPPA] = meshgrid(velocity_vec,kappa_vec);
% 
% 
% %% ============================================================
% % Velocity contribution
% %
% % kv -> 1       for v << v0
% % kv -> kv_min  for v >> v0
% % ============================================================
% 
% kv = kv_min + ...
%     (1-kv_min) ./ ...
%     (1 + exp((V-v0)/sv));
% 
% 
% %% ============================================================
% % Rear steering ratio law
% %
% % rr = f(kappa,v)
% %
% % Larger curvature  -> larger rear steering
% %
% % Higher velocity  -> smaller rear steering
% %
% % ============================================================
% 
% RR = -(1-exp(-KAPPA/kappa0)) .* kv;
% 
% 
% %% ============================================================
% % Saturation
% %
% % rr is restricted to [-1,0]
% % ============================================================
% 
% RR = max(min(RR,0),-1);
% 
% 
% %% ============================================================
% % Kinematic curvature constraint
% %
% % For a given rr, the maximum achievable curvature is:
% %
% % kappa_max(rr) =
% % sin((1-rr)*delta_max)
% % ----------------------
% % L*cos(rr*delta_max)
% %
% % Therefore a point is physically reachable only if:
% %
% % KAPPA <= kappa_max(RR)
% %
% % ============================================================
% 
% KAPPA_max_RR = ...
%     sin((1-RR).*delta_max) ./ ...
%     (L.*cos(RR.*delta_max));
% 
% 
% %% ============================================================
% % Remove physically unreachable combinations
% % ============================================================
% 
% unreachable = KAPPA > KAPPA_max_RR;
% 
% RR(unreachable) = NaN;
% 
% 
% %% ============================================================
% % Important curvature limits
% % ============================================================
% 
% % 2WS maximum curvature
% % rr = 0
% 
% kappa_max_2WS = ...
%     sin(delta_max) / ...
%     (L*cos(0*delta_max));
% 
% 
% % Absolute 4WS maximum curvature
% % rr = -1
% 
% kappa_max_4WS = ...
%     sin(2*delta_max) / ...
%     (L*cos(-delta_max));
% 
% 
% % Corresponding radii
% 
% Rmin_2WS = 1/kappa_max_2WS;
% 
% Rmin_4WS = 1/kappa_max_4WS;
% 
% 
% %% ============================================================
% % Surface
% % ============================================================
% 
% figure(7)
% clf
% 
% surf( ...
%     V*3.6, ...
%     KAPPA, ...
%     RR, ...
%     'EdgeColor','none', ...
%     'FaceAlpha',0.95)
% 
% hold on
% grid on
% box on
% 
% 
% %% ============================================================
% % 2WS maximum curvature boundary
% % ============================================================
% 
% plot3( ...
%     [0 30], ...
%     [kappa_max_2WS kappa_max_2WS], ...
%     [0 0], ...
%     'k--', ...
%     'LineWidth',2);
% 
% 
% %% ============================================================
% % Absolute 4WS maximum curvature boundary
% % ============================================================
% 
% plot3( ...
%     [0 30], ...
%     [kappa_max_4WS kappa_max_4WS], ...
%     [-1 -1], ...
%     'm--', ...
%     'LineWidth',2);
% 
% 
% %% ============================================================
% % Labels
% % ============================================================
% 
% xlabel('Velocity [km/h]')
% 
% ylabel('Reference curvature, \kappa_{ref} [1/m]')
% 
% zlabel('Rear steering ratio, rr')
% 
% 
% title({
%     'Rear steering ratio as a function of velocity and curvature'
%     sprintf('\\delta_{f,max}=%.2f rad, rr \\in [-1,0]',delta_max)
%     })
% 
% 
% %% ============================================================
% % Axis limits
% % ============================================================
% 
% view(40,30)
% 
% xlim([0 30])
% 
% ylim([0 kappa_max_4WS])
% 
% zlim([-1 0])
% 
% 
% %% ============================================================
% % Colormap
% % ============================================================
% 
% colormap(turbo)
% 
% cb = colorbar;
% 
% cb.Label.String = 'Rear steering ratio, rr';
% 
% clim([-1 0])






%% ============================================================
% Figure 8
%
% Optimal steering allocation
%
% Compare three different configurations:
%
%   Configuration 1:
%       Control variables: rr, delta_f
%
%       J =
%       wf1*(delta_f/delta_max)^2
%       +
%       wr1*rr^2
%
%
%   Configuration 2:
%       Control variables: rr, delta_f
%
%       J =
%       wf2*(delta_f/delta_max)^2
%       +
%       wr2*(rr*delta_f/delta_max)^2
%
%
%   Configuration 3:
%       Control variables: delta_f, delta_r
%
%       J =
%       wf3*(delta_f/delta_max)^2
%       +
%       wr3*(delta_r/delta_r_max)^2
%
%       rr = delta_r/delta_f
%
%
% Exact 4WS kinematic model:
%
%             sin((1-rr)*delta_f)
% kappa = ------------------------------
%              L*cos(rr*delta_f)
%
% Equivalent formulation using delta_f and delta_r:
%
%             sin(delta_f-delta_r)
% kappa = --------------------------
%                L*cos(delta_r)
%
% ============================================================

clc


% ============================================================
% Vehicle parameters
% ============================================================

L = 2;                  % Wheelbase [m]

delta_max = 0.7;        % Maximum front steering angle [rad]

delta_r_max = 0.7;      % Maximum rear steering angle [rad]


% ============================================================
% Optimization weights
% ============================================================

% Configuration 1

wf1 = 1;
wr1 = 0.6;


% Configuration 2

wf2 = 1;
wr2 = 2;


% Configuration 3

wf3 = 1;
wr3 = 2;


% ============================================================
% Parameter ranges for curvature surface
% ============================================================

N_rr    = 501;
N_delta = 501;

rr_vec = linspace(-1,0,N_rr);

delta_f_vec = linspace(0,delta_max,N_delta);

[RR,DELTA_F] = meshgrid(rr_vec,delta_f_vec);


% ============================================================
% Curvature surface
% ============================================================

KAPPA = ...
    sin((1-RR).*DELTA_F) ./ ...
    (L.*cos(RR.*DELTA_F));


% ============================================================
% Maximum curvature
% ============================================================

% 2WS maximum curvature

kappa_2WS_max = ...
    sin(delta_max) / ...
    (L*cos(0));


% Absolute 4WS maximum curvature

kappa_4WS_max = ...
    sin(2*delta_max) / ...
    (L*cos(-delta_max));


% ============================================================
% Curvature values for optimization
% ============================================================

N_kappa = 501;

kappa_vec = ...
    linspace(0,kappa_4WS_max,N_kappa);


% ============================================================
% Allocate optimal solutions
% ============================================================

% ------------------------------------------------------------
% Configuration 1
% ------------------------------------------------------------

rr_opt_1 = NaN(size(kappa_vec));

delta_opt_1 = NaN(size(kappa_vec));

J_opt_1 = NaN(size(kappa_vec));


% ------------------------------------------------------------
% Configuration 2
% ------------------------------------------------------------

rr_opt_2 = NaN(size(kappa_vec));

delta_opt_2 = NaN(size(kappa_vec));

J_opt_2 = NaN(size(kappa_vec));


% ------------------------------------------------------------
% Configuration 3
%
% Control variables:
%
% x(1) = delta_f
% x(2) = delta_r
% ------------------------------------------------------------

rr_opt_3 = NaN(size(kappa_vec));

delta_f_opt_3 = NaN(size(kappa_vec));

delta_r_opt_3 = NaN(size(kappa_vec));

J_opt_3 = NaN(size(kappa_vec));


% ============================================================
% Optimization options
% ============================================================

options = optimoptions('fmincon', ...
    'Algorithm','sqp', ...
    'Display','off', ...
    'OptimalityTolerance',1e-10, ...
    'ConstraintTolerance',1e-10);


% ============================================================
% OPTIMIZATION
% ============================================================

for j = 1:length(kappa_vec)

    kappa_ref = kappa_vec(j);


    % ========================================================
    % Initial guess for Configurations 1 and 2
    % ========================================================

    if kappa_ref <= kappa_2WS_max

        delta_initial = asin(L*kappa_ref);

        x0_rr = [0 delta_initial];

    else

        x0_rr = [-0.5 delta_max];

    end


    % ========================================================
    % CONFIGURATION 1
    %
    % Variables:
    %
    % x(1) = rr
    % x(2) = delta_f
    %
    % J =
    % wf1*(delta_f/delta_max)^2
    % +
    % wr1*rr^2
    % ========================================================

    objective_1 = @(x) ...
        wf1*(x(2)/delta_max)^2 + ...
        wr1*(x(1)/1)^2;


    nonlinear_constraint_rr = @(x) deal( ...
        [], ...
        sin((1-x(1))*x(2)) ./ ...
        (L*cos(x(1)*x(2))) ...
        - kappa_ref);


    [x_opt,Jval,exitflag] = ...
        fmincon( ...
            objective_1, ...
            x0_rr, ...
            [],[],[],[], ...
            [-1 0], ...
            [0 delta_max], ...
            nonlinear_constraint_rr, ...
            options);


    if exitflag > 0

        rr_opt_1(j) = x_opt(1);

        delta_opt_1(j) = x_opt(2);

        J_opt_1(j) = Jval;

    end


    % ========================================================
    % CONFIGURATION 2
    %
    % Variables:
    %
    % x(1) = rr
    % x(2) = delta_f
    %
    % J =
    % wf2*(delta_f/delta_max)^2
    % +
    % wr2*(rr*delta_f/delta_max)^2
    % ========================================================

    objective_2 = @(x) ...
        wf2*(x(2)/delta_max)^2 + ...
        wr2*(x(1)*x(2)/delta_max)^2;


    [x_opt,Jval,exitflag] = ...
        fmincon( ...
            objective_2, ...
            x0_rr, ...
            [],[],[],[], ...
            [-1 0], ...
            [0 delta_max], ...
            nonlinear_constraint_rr, ...
            options);


    if exitflag > 0

        rr_opt_2(j) = x_opt(1);

        delta_opt_2(j) = x_opt(2);

        J_opt_2(j) = Jval;

    end


    % ========================================================
    % CONFIGURATION 3
    %
    % Variables:
    %
    % x(1) = delta_f
    % x(2) = delta_r
    %
    % J =
    % wf3*(delta_f/delta_max)^2
    % +
    % wr3*(delta_r/delta_r_max)^2
    %
    % Rear steering is calculated directly.
    % rr is calculated afterwards:
    %
    % rr = delta_r/delta_f
    % ========================================================


    % --------------------------------------------------------
    % Initial guess
    % --------------------------------------------------------

    if kappa_ref <= kappa_2WS_max

        delta_f_initial = asin(L*kappa_ref);

        delta_r_initial = 0;

    else

        delta_f_initial = delta_max;

        delta_r_initial = -0.1;

    end


    x0_dr = ...
        [delta_f_initial delta_r_initial];


    % --------------------------------------------------------
    % Objective
    % --------------------------------------------------------

    objective_3 = @(x) ...
        wf3*(x(1)/delta_max)^2 + ...
        wr3*(x(2)/delta_r_max)^2;


    % --------------------------------------------------------
    % Exact curvature constraint
    %
    % kappa =
    %
    % sin(delta_f-delta_r)
    % ---------------------
    % L*cos(delta_r)
    % --------------------------------------------------------

    nonlinear_constraint_dr = @(x) deal( ...
        [], ...
        sin(x(1)-x(2)) ./ ...
        (L*cos(x(2))) ...
        - kappa_ref);


    % --------------------------------------------------------
    % Bounds
    %
    % delta_f:
    %     0 <= delta_f <= delta_max
    %
    % delta_r:
    %    -delta_r_max <= delta_r <= 0
    % --------------------------------------------------------

    lb_dr = ...
        [0 -delta_r_max];

    ub_dr = ...
        [delta_max 0];


    % --------------------------------------------------------
    % Solve
    % --------------------------------------------------------

    [x_opt,Jval,exitflag] = ...
        fmincon( ...
            objective_3, ...
            x0_dr, ...
            [],[],[],[], ...
            lb_dr, ...
            ub_dr, ...
            nonlinear_constraint_dr, ...
            options);


    % --------------------------------------------------------
    % Store solution
    % --------------------------------------------------------

    if exitflag > 0

        delta_f_opt_3(j) = x_opt(1);

        delta_r_opt_3(j) = x_opt(2);

        J_opt_3(j) = Jval;


        % ----------------------------------------------------
        % Calculate rear steering ratio
        % ----------------------------------------------------

        if abs(x_opt(1)) > 1e-10

            rr_opt_3(j) = ...
                x_opt(2) / x_opt(1);

        else

            rr_opt_3(j) = 0;

        end

    end

end


% ============================================================
% Verify curvature
% ============================================================

kappa_check_1 = ...
    sin((1-rr_opt_1).*delta_opt_1) ./ ...
    (L*cos(rr_opt_1.*delta_opt_1));


kappa_check_2 = ...
    sin((1-rr_opt_2).*delta_opt_2) ./ ...
    (L*cos(rr_opt_2.*delta_opt_2));


kappa_check_3 = ...
    sin(delta_f_opt_3-delta_r_opt_3) ./ ...
    (L*cos(delta_r_opt_3));


% ============================================================
% FIGURE
% ============================================================

figure(8)
clf

hold on
grid on
box on


% ============================================================
% Curvature surface
% ============================================================

surf( ...
    RR, ...
    KAPPA, ...
    DELTA_F, ...
    'EdgeColor','none', ...
    'FaceAlpha',0.50);


% ============================================================
% Optimal solution - Configuration 1
% ============================================================

plot3( ...
    rr_opt_1, ...
    kappa_vec, ...
    delta_opt_1, ...
    'k', ...
    'LineWidth',3);


% ============================================================
% Optimal solution - Configuration 2
% ============================================================

plot3( ...
    rr_opt_2, ...
    kappa_vec, ...
    delta_opt_2, ...
    'r', ...
    'LineWidth',3);


% ============================================================
% Optimal solution - Configuration 3
%
% Configuration 3 uses:
%
% delta_f = delta_f_opt_3
% rr      = rr_opt_3
%
% Therefore it can be plotted on the same
% (rr,kappa,delta_f) coordinate system.
% ============================================================

plot3( ...
    rr_opt_3, ...
    kappa_vec, ...
    delta_f_opt_3, ...
    'g', ...
    'LineWidth',3);


% ============================================================
% 2WS solution
% ============================================================

idx_2WS = ...
    kappa_vec <= kappa_2WS_max;


delta_2WS_curve = ...
    asin(L*kappa_vec(idx_2WS));


plot3( ...
    zeros(size(kappa_vec(idx_2WS))), ...
    kappa_vec(idx_2WS), ...
    delta_2WS_curve, ...
    'w--', ...
    'LineWidth',2);


% ============================================================
% 2WS maximum curvature boundary
% ============================================================

plot3( ...
    [-1 0], ...
    [kappa_2WS_max kappa_2WS_max], ...
    [delta_max delta_max], ...
    'b--', ...
    'LineWidth',2);


% ============================================================
% Labels
% ============================================================

xlabel('Rear steering ratio, rr')

ylabel('Curvature, \kappa [1/m]')

zlabel('Front steering angle, \delta_f [rad]')


title({
    'Optimal 4WS steering allocation'
    sprintf('Configuration 1: w_f=%.2f, w_r=%.2f',wf1,wr1)
    sprintf('Configuration 2: w_f=%.2f, w_r=%.2f',wf2,wr2)
    sprintf('Configuration 3: w_f=%.2f, w_r=%.2f',wf3,wr3)
    })


% ============================================================
% Limits
% ============================================================

xlim([-1 0])

ylim([0 kappa_4WS_max])

zlim([0 delta_max])


% ============================================================
% View
% ============================================================

view(45,30)


% ============================================================
% Colormap
% ============================================================

colormap(turbo)

cb = colorbar;

cb.Label.String = ...
    'Front steering angle, \delta_f [rad]';

clim([0 delta_max]);


% ============================================================
% Legend
% ============================================================

legend( ...
    'Kinematic surface', ...
    sprintf('Configuration 1: w_f=%.2f, w_r=%.2f',wf1,wr1), ...
    sprintf('Configuration 2: w_f=%.2f, w_r=%.2f',wf2,wr2), ...
    sprintf('Configuration 3: w_f=%.2f, w_r=%.2f',wf3,wr3), ...
    '2WS solution', ...
    '2WS maximum curvature', ...
    'Location','best');


%% ============================================================
% Save LUT1 - Configuration 1
%
% LUT mapping:
%   kappa_ref -> optimal rear steering ratio rr
% ============================================================

k_ref_LUT = kappa_vec(:);
rr_LUT = rr_opt_1(:);
delta_f_LUT = delta_opt_1(:); 

% Remove invalid entries, if any
valid = isfinite(k_ref_LUT) & isfinite(rr_LUT) & isfinite(delta_f_LUT);

k_ref_LUT = k_ref_LUT(valid);
rr_LUT = rr_LUT(valid);
delta_f_LUT = delta_f_LUT(valid);

% Save LUT
save('/home/andrea-ricetti/Documenti/MATLAB/LUT/LUT_rr_computation.mat', ...
    'k_ref_LUT', ...
    'rr_LUT', ...
    'delta_f_LUT');







%% ============================================================
% Figure 11
%
% Optimal steering allocation
%
%
%   Configuration 4:
%       Control variables: rr, delta_f
%
%       J =
%       wf1*(delta_f/delta_max)^2
%       +
%       wr1*rr^2
%
%
% Exact 4WS kinematic model:
%
%             sin((1-rr)*delta_f)
% kappa_rear = ------------------------------
%              L*cos(delta_f)
%
%
% ============================================================

clc


% ============================================================
% Vehicle parameters
% ============================================================

L = 2;                  % Wheelbase [m]

delta_max = 0.7;        % Maximum front steering angle [rad]

delta_r_max = 0.7;      % Maximum rear steering angle [rad]


% ============================================================
% Optimization weights
% ============================================================

% Configuration 1

wf1 = 1;
wr1 = 0.4;

% ============================================================
% Parameter ranges for curvature surface
% ============================================================

N_rr    = 501;
N_delta = 501;

rr_vec = linspace(-1,0,N_rr);

delta_f_vec = linspace(0,delta_max,N_delta);

[RR,DELTA_F] = meshgrid(rr_vec,delta_f_vec);


% ============================================================
% Curvature surface
% ============================================================

KAPPA = ...
    sin((1-RR).*DELTA_F) ./ ...
    (L.*cos(DELTA_F));


% ============================================================
% Maximum curvature
% ============================================================

% 2WS maximum curvature

kappa_2WS_max = ...
    tan(delta_max) / ...
    (L*cos(0));


% Absolute 4WS maximum curvature

kappa_4WS_max = ...
    sin(2*delta_max) / ...
    (L*cos(delta_max));


% ============================================================
% Curvature values for optimization
% ============================================================

N_kappa = 501;

kappa_vec = ...
    linspace(0,kappa_4WS_max,N_kappa);


% ============================================================
% Allocate optimal solutions
% ============================================================

% ------------------------------------------------------------
% Configuration 4
% ------------------------------------------------------------

rr_opt_1 = NaN(size(kappa_vec));

delta_opt_1 = NaN(size(kappa_vec));

J_opt_1 = NaN(size(kappa_vec));


% ============================================================
% Optimization options
% ============================================================

options = optimoptions('fmincon', ...
    'Algorithm','sqp', ...
    'Display','off', ...
    'OptimalityTolerance',1e-10, ...
    'ConstraintTolerance',1e-10);


% ============================================================
% OPTIMIZATION
% ============================================================

for j = 1:length(kappa_vec)

    kappa_ref = kappa_vec(j);


    % ========================================================
    % Initial guess for Configurations 1 and 2
    % ========================================================

    if kappa_ref <= kappa_2WS_max

        delta_initial = atan(L*kappa_ref);

        x0_rr = [0 delta_initial];

    else

        x0_rr = [-0.5 delta_max];

    end


    % ========================================================
    % CONFIGURATION 4
    %
    % Variables:
    %
    % x(1) = rr
    % x(2) = delta_f
    %
    % J =
    % wf1*(delta_f/delta_max)^2
    % +
    % wr1*rr^2
    % ========================================================

    objective_1 = @(x) ...
        wf1*(x(2)/delta_max)^2 + ...
        wr1*(x(1)/1)^2;


    nonlinear_constraint_rr = @(x) deal( ...
        [], ...
        sin((1-x(1))*x(2)) ./ ...
        (L*cos(x(2))) ...
        - kappa_ref);


    [x_opt,Jval,exitflag] = ...
        fmincon( ...
            objective_1, ...
            x0_rr, ...
            [],[],[],[], ...
            [-1 0], ...
            [0 delta_max], ...
            nonlinear_constraint_rr, ...
            options);


    if exitflag > 0

        rr_opt_1(j) = x_opt(1);

        delta_opt_1(j) = x_opt(2);

        J_opt_1(j) = Jval;

    end
end


% ============================================================
% Verify curvature
% ============================================================

kappa_check_1 = ...
    sin((1-rr_opt_1).*delta_opt_1) ./ ...
    (L*cos(delta_opt_1));


% ============================================================
% FIGURE
% ============================================================

figure(11)
clf

hold on
grid on
box on


% ============================================================
% Curvature surface
% ============================================================

surf( ...
    RR, ...
    KAPPA, ...
    DELTA_F, ...
    'EdgeColor','none', ...
    'FaceAlpha',0.50);


% ============================================================
% Optimal solution - Configuration 1
% ============================================================

plot3( ...
    rr_opt_1, ...
    kappa_vec, ...
    delta_opt_1, ...
    'k', ...
    'LineWidth',3);
% ============================================================
% 2WS solution
% ============================================================

idx_2WS = ...
    kappa_vec <= kappa_2WS_max;


delta_2WS_curve = ...
    asin(L*kappa_vec(idx_2WS));


plot3( ...
    zeros(size(kappa_vec(idx_2WS))), ...
    kappa_vec(idx_2WS), ...
    delta_2WS_curve, ...
    'w--', ...
    'LineWidth',2);


% ============================================================
% 2WS maximum curvature boundary
% ============================================================

plot3( ...
    [-1 0], ...
    [kappa_2WS_max kappa_2WS_max], ...
    [delta_max delta_max], ...
    'b--', ...
    'LineWidth',2);


% ============================================================
% Labels
% ============================================================

xlabel('Rear steering ratio, rr')

ylabel('Curvature, \kappa [1/m]')

zlabel('Front steering angle, \delta_f [rad]')


title({
    'Optimal 4WS steering allocation based on K_ref_rear'
    sprintf('Configuration 4: w_f=%.2f, w_r=%.2f',wf1,wr1)
    })


% ============================================================
% Limits
% ============================================================

xlim([-1 0])

ylim([0 kappa_4WS_max])

zlim([0 delta_max])


% ============================================================
% View
% ============================================================

view(45,30)


% ============================================================
% Colormap
% ============================================================

colormap(turbo)

cb = colorbar;

cb.Label.String = ...
    'Front steering angle, \delta_f [rad]';

clim([0 delta_max]);


% ============================================================
% Legend
% ============================================================

legend( ...
    'Kinematic surface', ...
    sprintf('Configuration 4: w_f=%.2f, w_r=%.2f',wf1,wr1), ...
    '2WS solution', ...
    '2WS maximum curvature', ...
    'Location','best');




% %% ============================================================
% %% Figure 9 rr = f(k_ref, v_x)

% %
% % Optimal rear steering ratio:
% %
% %               rr = f(kappa_ref, velocity)
% %
% % Optimization:
% %
% % min J(delta_f,rr)
% %
% % J =
% % wf * (delta_f/delta_max)^2
% % +
% % wr * (rr/rr_max)^2
% %
% % subject to:
% %
% % kappa =
% % sin((1-rr)*delta_f)
% % ----------------------
% % L*cos(rr*delta_f)
% %
% % and:
% %
% % a_y = v^2 * kappa_ref <= a_y_max
% %
% % For a_y > a_y_max:
% % use the last admissible rr value for each velocity.
% %
% % ============================================================
% 
% clc
% 
% 
% %% ============================================================
% % Vehicle parameters
% % ============================================================
% 
% L = 2;                  % Wheelbase [m]
% 
% delta_max = 0.7;        % Maximum front steering angle [rad]
% 
% rr_max = 1;
% 
% 
% % ============================================================
% % Lateral acceleration limit
% % ============================================================
% 
% a_y_max = 1;            % Maximum lateral acceleration [m/s^2]
% 
% 
% % ============================================================
% % Optimization weights
% % ============================================================
% 
% wf = 1;
% 
% wr = 0.6;
% 
% 
% % ============================================================
% % Parameter ranges
% % ============================================================
% 
% N_v     = 101;
% N_kappa = 101;
% 
% 
% velocity_vec = ...
%     linspace(0,30/3.6,N_v);
% 
% 
% % ============================================================
% % Maximum theoretically reachable curvature
% %
% % rr = -1
% % delta_f = delta_max
% % ============================================================
% 
% kappa_max_4WS = ...
%     sin(2*delta_max) / ...
%     (L*cos(-delta_max));
% 
% 
% kappa_vec = ...
%     linspace(0,kappa_max_4WS,N_kappa);
% 
% 
% [V,KAPPA] = ...
%     meshgrid(velocity_vec,kappa_vec);
% 
% 
% % ============================================================
% % Lateral acceleration
% %
% % a_y = v^2 * kappa
% % ============================================================
% 
% ACC_y = ...
%     V.^2 .* KAPPA;
% 
% 
% % ============================================================
% % Feasibility condition
% %
% % a_y <= a_y_max
% % ============================================================
% 
% feasible_ay = ...
%     ACC_y <= a_y_max;
% 
% 
% % ============================================================
% % Allocate optimal solution
% % ============================================================
% 
% RR_opt = ...
%     NaN(size(KAPPA));
% 
% DELTA_opt = ...
%     NaN(size(KAPPA));
% 
% J_opt = ...
%     NaN(size(KAPPA));
% 
% 
% % ============================================================
% % Optimization options
% % ============================================================
% 
% options = optimoptions('fmincon', ...
%     'Algorithm','sqp', ...
%     'Display','off', ...
%     'OptimalityTolerance',1e-8, ...
%     'ConstraintTolerance',1e-8);
% 
% 
% % ============================================================
% % Optimization
% % ============================================================
% 
% for j = 1:N_kappa
% 
%     for i = 1:N_v
% 
%         kappa_ref = KAPPA(j,i);
% 
%         velocity = V(j,i);
% 
% 
%         % ----------------------------------------------------
%         % Reject excessive lateral acceleration
%         % ----------------------------------------------------
% 
%         if velocity > 0 && ...
%                 velocity^2 * kappa_ref > a_y_max
% 
%             continue
% 
%         end
% 
% 
%         % ----------------------------------------------------
%         % Zero curvature
%         % ----------------------------------------------------
% 
%         if abs(kappa_ref) < 1e-10
% 
%             RR_opt(j,i) = 0;
% 
%             DELTA_opt(j,i) = 0;
% 
%             J_opt(j,i) = 0;
% 
%             continue
% 
%         end
% 
% 
%         % ----------------------------------------------------
%         % Initial guess
%         % ----------------------------------------------------
% 
%         if kappa_ref <= ...
%                 sin(delta_max)/L
% 
%             % 2WS solution
% 
%             delta_initial = ...
%                 asin(L*kappa_ref);
% 
%             x0 = ...
%                 [0 delta_initial];
% 
%         else
% 
%             % Start from a configuration using rear steering
% 
%             x0 = ...
%                 [-0.5 delta_max];
% 
%         end
% 
% 
%         % ----------------------------------------------------
%         % Objective function
%         %
%         % J =
%         %
%         % wf*(delta_f/delta_max)^2
%         %
%         % +
%         %
%         % wr*(rr/rr_max)^2
%         %
%         % ----------------------------------------------------
% 
%         objective = @(x) ...
%             wf*(x(2)/delta_max)^2 + ...
%             wr*(x(1)/rr_max)^2;
% 
% 
%         % ----------------------------------------------------
%         % Exact curvature constraint
%         % ----------------------------------------------------
% 
%         nonlinear_constraint = @(x) deal( ...
%             [], ...
%             sin((1-x(1))*x(2)) ./ ...
%             (L*cos(x(1)*x(2))) ...
%             - kappa_ref );
% 
% 
%         % ----------------------------------------------------
%         % Bounds
%         % ----------------------------------------------------
% 
%         lb = [-1 0];
% 
%         ub = [0 delta_max];
% 
% 
%         % ----------------------------------------------------
%         % Solve
%         % ----------------------------------------------------
% 
%         [x_opt,Jval,exitflag] = ...
%             fmincon( ...
%                 objective, ...
%                 x0, ...
%                 [],[],[],[], ...
%                 lb,ub, ...
%                 nonlinear_constraint, ...
%                 options);
% 
% 
%         % ----------------------------------------------------
%         % Store solution
%         % ----------------------------------------------------
% 
%         if exitflag > 0
% 
%             RR_opt(j,i) = ...
%                 x_opt(1);
% 
%             DELTA_opt(j,i) = ...
%                 x_opt(2);
% 
%             J_opt(j,i) = ...
%                 Jval;
% 
%         end
% 
%     end
% 
% end
% 
% 
% % ============================================================
% % Verify generated curvature
% % ============================================================
% 
% KAPPA_check = ...
%     sin((1-RR_opt).*DELTA_opt) ./ ...
%     (L*cos(RR_opt.*DELTA_opt));
% 
% 
% % ============================================================
% % Check optimization error
% % ============================================================
% 
% error_kappa = ...
%     abs(KAPPA_check-KAPPA);
% 
% 
% % ============================================================
% % Remove invalid optimization points
% % ============================================================
% 
% invalid_optimization = ...
%     error_kappa > 1e-6;
% 
% 
% RR_opt(invalid_optimization) = NaN;
% 
% DELTA_opt(invalid_optimization) = NaN;
% 
% J_opt(invalid_optimization) = NaN;
% 
% 
% % ============================================================
% % Create final LUT
% %
% % For each velocity:
% %
% % - use optimized rr while a_y <= a_y_max
% % - beyond the limit, keep the last admissible rr
% %
% % ============================================================
% 
% RR_LUT = ...
%     RR_opt;
% 
% DELTA_LUT = ...
%     DELTA_opt;
% 
% J_LUT = ...
%     J_opt;
% 
% 
% for i = 1:N_v
% 
%     % --------------------------------------------------------
%     % Find points admissible according to lateral acceleration
%     % and successfully optimized
%     % --------------------------------------------------------
% 
%     valid_idx = ...
%         find( ...
%         feasible_ay(:,i) & ...
%         ~isnan(RR_opt(:,i)) );
% 
% 
%     % --------------------------------------------------------
%     % If no valid point exists, skip
%     % --------------------------------------------------------
% 
%     if isempty(valid_idx)
% 
%         continue
% 
%     end
% 
% 
%     % --------------------------------------------------------
%     % Last admissible point
%     % --------------------------------------------------------
% 
%     last_valid_idx = ...
%         valid_idx(end);
% 
% 
%     rr_last = ...
%         RR_opt(last_valid_idx,i);
% 
%     delta_last = ...
%         DELTA_opt(last_valid_idx,i);
% 
%     J_last = ...
%         J_opt(last_valid_idx,i);
% 
% 
%     % --------------------------------------------------------
%     % Points above lateral acceleration limit
%     % --------------------------------------------------------
% 
%     saturated_idx = ...
%         find( ...
%         ~feasible_ay(:,i) );
% 
% 
%     % --------------------------------------------------------
%     % Saturate rr to last admissible value
%     % --------------------------------------------------------
% 
%     RR_LUT(saturated_idx,i) = ...
%         rr_last;
% 
%     DELTA_LUT(saturated_idx,i) = ...
%         delta_last;
% 
%     J_LUT(saturated_idx,i) = ...
%         J_last;
% 
% end
% 
% 
% % ============================================================
% % Important curvature limits
% % ============================================================
% 
% kappa_max_2WS = ...
%     sin(delta_max) / L;
% 
% 
% Rmin_2WS = ...
%     1/kappa_max_2WS;
% 
% 
% Rmin_4WS = ...
%     1/kappa_max_4WS;
% 
% 
% % ============================================================
% % Velocity-dependent lateral acceleration boundary
% %
% % kappa_max(v) = a_y_max / v^2
% % ============================================================
% 
% kappa_max_ay = ...
%     NaN(size(velocity_vec));
% 
% 
% nonzero_velocity = ...
%     velocity_vec > 0;
% 
% 
% kappa_max_ay(nonzero_velocity) = ...
%     a_y_max ./ ...
%     velocity_vec(nonzero_velocity).^2;
% 
% 
% % At zero velocity, the lateral acceleration constraint
% % does not impose a curvature limit.
% 
% kappa_max_ay(~nonzero_velocity) = ...
%     kappa_max_4WS;
% 
% 
% % ============================================================
% % Figure
% % ============================================================
% 
% figure(9)
% clf
% 
% surf( ...
%     V*3.6, ...
%     KAPPA, ...
%     RR_LUT, ...
%     ACC_y, ...
%     'EdgeColor','none', ...
%     'FaceAlpha',0.95);
% 
% hold on
% grid on
% box on
% 
% 
% % ============================================================
% % 2WS maximum curvature boundary
% % ============================================================
% 
% plot3( ...
%     [0 30], ...
%     [kappa_max_2WS kappa_max_2WS], ...
%     [0 0], ...
%     'k--', ...
%     'LineWidth',2);
% 
% 
% % ============================================================
% % Maximum 4WS curvature boundary
% % ============================================================
% 
% plot3( ...
%     [0 30], ...
%     [kappa_max_4WS kappa_max_4WS], ...
%     [-1 -1], ...
%     'm--', ...
%     'LineWidth',2);
% 
% 
% % ============================================================
% % Lateral acceleration boundary
% % ============================================================
% 
% valid_boundary = ...
%     kappa_max_ay <= kappa_max_4WS;
% 
% 
% plot3( ...
%     velocity_vec(valid_boundary)*3.6, ...
%     kappa_max_ay(valid_boundary), ...
%     zeros(1,sum(valid_boundary)), ...
%     'r-', ...
%     'LineWidth',2.5);
% 
% 
% % ============================================================
% % Labels
% % ============================================================
% 
% xlabel('Velocity [km/h]')
% 
% ylabel('Reference curvature, \kappa_{ref} [1/m]')
% 
% zlabel('Optimal rear steering ratio, rr^*')
% 
% 
% title({
%     'Optimal rear steering ratio'
%     sprintf( ...
%     'w_f = %.1f, w_r = %.1f, a_{y,max} = %.1f m/s^2', ...
%     wf,wr,a_y_max)
%     })
% 
% 
% % ============================================================
% % Limits
% % ============================================================
% 
% xlim([0 30])
% 
% ylim([0 kappa_max_4WS])
% 
% zlim([-1 0])
% 
% 
% % ============================================================
% % View
% % ============================================================
% 
% view(40,30)
% 
% 
% % ============================================================
% % Colormap
% % ============================================================
% 
% colormap(turbo)
% 
% cb = colorbar;
% 
% cb.Label.String = ...
%     'Lateral acceleration, a_y [m/s^2]';
% 
% clim([0 a_y_max]);
% 
% 
% % ============================================================
% % Save LUT
% %
% % rr = f(velocity, kappa_ref)
% % ============================================================
% 
% velocity_BP = ...
%     velocity_vec;          % [m/s]
% 
% kappa_rr_BP = ...
%     kappa_vec;             % [1/m]
% 
% rr_LUT = ...
%     RR_LUT;
% 
% 
% save( ...
%     '/home/andrea-ricetti/Documenti/MATLAB/LUT/LUT_rr_computation.mat', ...
%     'velocity_BP', ...
%     'kappa_rr_BP', ...
%     'rr_LUT');



%% ============================================================
% Figure 10
%
% Stanley front steering angle as a function of:
%   - longitudinal velocity vx
%   - lateral error ey
%
% front_steer = 15 * cross_track_term
%
% cross_track_term =
% atan2(k_gain*ey, vx + k_soft)
%
% ============================================================

% clc
% clear
% 
% 
% 
% % ============================================================
% % Stanley parameters
% % ============================================================
% 
% k_gain = 0.8;          % Lateral error gain
% k_soft = 1.5;          % Softening velocity [m/s]
% 
% stanley_gain = 15;     % Gain applied to cross-track term
% 
% 
% % ============================================================
% % Vehicle / steering parameters
% % ============================================================
% 
% delta_max = 0.7;       % Maximum steering angle [rad]
% 
% 
% % ============================================================
% % Parameter ranges
% % ============================================================
% 
% N_v  = 501;
% N_ey = 501;
% 
% velocity_vec = ...
%     linspace(0,30/3.6,N_v);       % [m/s]
% 
% lateral_error_vec = ...
%     linspace(-1,1,N_ey);          % [m]
% 
% 
% [V, EY] = meshgrid(velocity_vec,lateral_error_vec);
% 
% 
% % ============================================================
% % Cross-track term
% % ============================================================
% 
% cross_track_term = atan2( ...
%     k_gain .* EY, ...
%     V + k_soft);
% 
% 
% % ============================================================
% % Stanley steering command
% % ============================================================
% 
% DELTA_2WS = ...
%     stanley_gain .* cross_track_term;
% 
% 
% % ============================================================
% % Saturated steering command
% % ============================================================
% 
% DELTA_2WS_sat = ...
%     max(min(DELTA_2WS,delta_max),-delta_max);
% 
% 
% % ============================================================
% % Surface - SATURATED
% % ============================================================
% 
% figure(10)
% clf
% 
% surf( ...
%     V*3.6, ...
%     EY, ...
%     DELTA_2WS_sat, ...
%     'EdgeColor','none', ...
%     'FaceAlpha',0.95);
% 
% hold on
% grid on
% box on
% 
% xlabel('Longitudinal velocity, v_x [km/h]')
% ylabel('Lateral error, e_y [m]')
% zlabel('\delta_{2WS} [rad]')
% 
% title({
%     'Stanley steering command'
%     sprintf('\\delta_{2WS}=15 atan2(k_{gain}e_y,v_x+k_{soft})')
%     })
% 
% view(40,30)
% 
% xlim([0 30])
% ylim([-1 1])
% 
% colormap(turbo)
% 
% cb = colorbar;
% cb.Label.String = '\delta_{2WS} [rad]';
% 
% 
% % ============================================================
% % Steering limits
% % ============================================================
% 
% zlim([min(DELTA_2WS_sat(:)) max(DELTA_2WS_sat(:))])