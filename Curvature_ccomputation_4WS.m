% clc
% clear all
% close all
% 
% L = 2;                      % wheelbase [m]
% delta_f = 0.7;              % max steering angle [rad]
% 
% rr = linspace(-1,1,101);    % rear-steer ratio
% 
% curvature = zeros(size(rr));
% R = zeros(size(rr));        % radius of curvature
% 
% for i = 1:length(rr)
%     curvature(i) = sin((1-rr(i))*delta_f)/(L*cos(delta_f));
%     R(i) = 1/curvature(i);
% end
% 
% figure
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


clc
clear all
close all

L = 2;                      % wheelbase [m]
delta_f = 0.7;              % 2WS steering angle [rad]

rr = linspace(-1,1,101);    % rear-steer ratio

curvature = zeros(size(rr));
R = zeros(size(rr));        % radius of curvature
gain_4WS = zeros(size(rr)); % required gain


%% Curvature and radius with fixed delta_f
for i = 1:length(rr)

    curvature(i) = sin((1-rr(i))*delta_f)/(L*cos(delta_f));
    R(i) = 1/curvature(i);

end


clc
clear all
close all

%% Vehicle parameters
L = 2;                  % wheelbase [m]

delta_2WS = 0.7;        % steering command from Stanley 2WS [rad]

%% Rear steering ratio range
rr = linspace(-1,1,101);

gain4WS = zeros(size(rr));


%% Solve gain4WS for each rr

for i = 1:length(rr)

    % Equation:
    % tan(delta_2WS) =
    % sin((1-rr)*gain*delta_2WS)/cos(gain*delta_2WS)

    eq = @(gain) ...
        sin((1-rr(i))*gain*delta_2WS) / ...
        cos(gain*delta_2WS) ...
        - tan(delta_2WS);

    v0 = eq(0);
    v1 = eq(1);
    if v0 * v1 < 0
        gain4WS(i) = fzero(eq, [0 1]);
    else
        % Nessuna radice nell'intervallo [0,1]: usare valore di fallback o NaN
        gain4WS(i) = NaN;
    end


end


%% Plot

figure
plot(rr,gain4WS,'LineWidth',2)
grid on
hold on

xlabel('Rear steering ratio rr')
ylabel('gain_{4WS}')
title(['Relation rr - gain_{4WS}, \delta_{2WS}=',...
    num2str(delta_2WS),' rad'])

xlim([-1 1])
ylim([0 1])
