clc
clear all
close all

L = 2;                      % wheelbase [m]
delta_max = 0.7;

rr = linspace(-1,1,101);    % rear-steer ratio

curvature = zeros(size(rr));
R = zeros(size(rr));        % radius of curvature
gain_4WS = zeros(size(rr)); % required gain


%% Curvature and radius with fixed delta_f
for i = 1:length(rr)

    curvature(i) = sin((1-rr(i))*delta_max)/(L*cos(delta_max));
    R(i) = 1/curvature(i);

end

figure(1)
plot(rr, R, 'b', 'LineWidth',2)
hold on
grid on

% Evidenzia rr = 0
idx0 = find(rr==0);

plot(rr(idx0), R(idx0), 'ro', ...
    'MarkerSize',10, ...
    'MarkerFaceColor','r')

text(rr(idx0), R(idx0), ...
    sprintf('  rr = 0\n  R = %.2f m', R(idx0)), ...
    'FontSize',10)

% Evidenzia minimo R
[Rmin, idxMin] = min(R);

plot(rr(idxMin), Rmin, 'ks', ...
    'MarkerSize',10, ...
    'MarkerFaceColor','g')

text(rr(idxMin), Rmin, ...
    sprintf('  R_{min}=%.2f m\n  rr=%.2f', Rmin, rr(idxMin)), ...
    'FontSize',10)

xlabel('Rear-steer ratio, rr')
ylabel('Radius of curvature, R [m]')
title('Effect of rear steering ratio on turning radius (gain4WS = 1, delta_f(max) = 0.7 rad)')
legend('R(rr)','2WS case (rr=0)','Minimum radius')



%% Figure 2 and 4
N = 10;                                 % N° of reference radii of curvature

Rmin_2WS = L/tan(delta_max);
Rmax = 10;

delta_2WS_vec = zeros(1,N);
delta_2WS_low_vec = zeros(1,N);

rr = zeros(101, N);           % rr: 101 righe (campioni) x N colonne (R_ref)
gain4WS = zeros(101, N);      % gain4WS: stessa dimensione di rr
gain4WS_low = zeros(101,N);

R_ref = linspace(Rmin_2WS, Rmax, N);
R_ref_low = linspace(Rmin,Rmin_2WS,N);



%% Rear steering ratio range

for j = 1:N    
    delta_2WS = min(atan(L/R_ref(j)), delta_max);
    delta_2WS_low = delta_max;

    % Memorizza lo sterzo equivalente 2WS
    delta_2WS_vec(j) = delta_2WS; 
    delta_2WS_low_vec(j) = delta_2WS_low;

    R_low = R_ref_low(j);

    rr(:,j) = linspace(-1,1,101)';

    % Solve gain4WS for each rr
    for i = 1:length(rr(:,j))
        % Equation:
        % tan(delta_2WS) =
        % sin((1-rr)*gain*delta_2WS)/cos(gain*delta_2WS)
    
        eq1 = @(gain) ...
            sin((1-rr(i,j))*gain*delta_2WS) / ...
            cos(gain*delta_2WS) ...
            - tan(delta_2WS);

        eq2 = @(gain) ...
            sin((1-rr(i,j))*gain*delta_max) ...
            /(L*cos(gain*delta_max)) ...
            - 1/R_low;
    
        v01 = eq1(0);
        v11 = eq1(1.5);
        if v01 * v11 < 0
            gain4WS(i,j) = fzero(eq1, [0 1.5]);
        else
            % Nessuna radice nell'intervallo [0,1.5]: usare valore di fallback o NaN
            gain4WS(i,j) = NaN;
        end


        v02 = eq2(0);
        v12 = eq2(1.5);
        if v02 * v12 < 0
            gain4WS_low(i,j) = fzero(eq2, [0 1.5]);
        else
            % Nessuna radice nell'intervallo [0,1.5]: usare valore di fallback o NaN
            gain4WS_low(i,j) = NaN;
        end

    end
end

%% Plot

figure(2)
clf
hold on
grid on

cmap = turbo(N);

for j = 1:N
    plot(rr(:,j), gain4WS(:,j), ...
        'Color', cmap(j,:), ...
        'LineWidth',2);
end

xlabel('Rear steering ratio, rr')
ylabel('gain_{4WS}')

xlim([-1 0])
ylim([0 1.5])

%% Colorbar

colormap(cmap)

cb = colorbar;

caxis([1 N])

cb.Ticks = 1:N;

labels = strings(1,N);

for j = 1:N

    labels(j) = sprintf('\\delta_{2WS}=%.2f rad, R_{ref}=%.2f m', ...
        delta_2WS_vec(j), R_ref(j));

end

cb.TickLabels = labels;

title({
    'Relation between rr and gain_{4WS}'
    sprintf('(valid up to \\delta_{2WS,max}=%.2f rad, R_{min}=%.2f m)', ...
    delta_max, Rmin_2WS)
    })


%% ============================================================
% Figure 4
% gain4WS(rr, delta2WS)
% ============================================================

figure(4)
clf

% Mesh della superfice
[RR, RADIUS] = meshgrid(rr(:,1), R_ref);

[RR, RADIUS_LOW] = meshgrid(rr(:,1), R_ref_low);

% gain4WS ha dimensioni:
%   101 x N
% mentre meshgrid restituisce:
%   N x 101
% quindi trasponiamo gain4WS

surf(RR, RADIUS, gain4WS', ...
    'EdgeColor','none', ...
    'FaceAlpha',0.95);
hold on
surf(RR,RADIUS_LOW,gain4WS_low',  ...
    'EdgeColor','none', ...
    'FaceAlpha',0.95);
grid on
box on

% Insert lines of figure 2
% cmap = turbo(N);
% 
% for j = 1:N
% 
%     valid = ~isnan(gain4WS(:,j));
% 
%     plot3( ...
%         rr(valid,j), ...
%         R_ref(j)*ones(sum(valid),1), ...
%         gain4WS(valid,j), ...
%         'Color',cmap(j,:), ...
%         'LineWidth',2);
% 
% end

xlabel('Rear steering ratio, rr')
ylabel('Reference turning radius, R_{ref} [m]')
zlabel('gain_{4WS}')

title({
    'Front steering reduction gain'
    sprintf('(valid up to \\delta_{2WS,max}=%.2f rad)',delta_max)
    })

view(40,30)

xlim([-1 0])
ylim([R_ref_low(1) R_ref(end)])
zlim([0 1])

colormap(turbo)

cb = colorbar;
cb.Label.String = 'gain_{4WS}';

clim([0.5 1])


%%================================
% Figure 3
%
%=================================

Rsurf = repmat(R_ref, length(rr(:,1)), 1);   % 101 x N

figure(3)
clf

Rsurf = repmat(R_ref,length(rr(:,1)),1);

surf(rr, gain4WS, Rsurf,...
    'EdgeColor','none',...
    'FaceAlpha',0.9)

hold on
grid on
box on

xlabel('Rear steering ratio, rr')
ylabel('gain_{4WS}')
zlabel('Reference turning radius, R_{ref} [m]')

title({
    'Reference turning radius'
    sprintf('(valid up to \\delta_{2WS,max}=%.2f rad)',delta_max)
    })

view(40,30)

xlim([-1 0])
ylim([0 1.2])
zlim([R_ref(1) R_ref(end)])

colormap(turbo)

cb = colorbar;
cb.Label.String = 'Reference turning radius [m]';

clim([R_ref(1) R_ref(end)])