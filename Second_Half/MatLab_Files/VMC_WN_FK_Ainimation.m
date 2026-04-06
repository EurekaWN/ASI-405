%% ==========================================================
%  Animate FK with phi1 and phi2 trajectories.
%  Axis limits are computed from all linkage points O..G.
%% ==========================================================
clear; clc; close all;

%% ===== parameters =====
P.L1 = 215.00;
P.L2 = 90.00;
P.L3 = 108.00;
P.L4 = 108.00;
P.L5 = 69.00;
P.L6 = 125.00;
P.L7 = 201.94;

P.k_a5 = 16.44;
P.a7   = 21.21;
P.k163 = 163.51; 

%% ===== choose how phi1, phi2 vary =====
T = 24;          % seconds
N = 360;        % frames
t = linspace(0, T, N);

% Reference motion trajectory (edit as needed)
phi1_traj = 30 + 20*sin(2*pi*0.35*t);     % deg
phi2_traj = 50 + 15*cos(2*pi*0.25*t);     % deg

%% ===== pre-scan axis limits using ALL points =====
names = {'O','A','B','C','D','E','F','G'};

xmin = +inf; xmax = -inf;
ymin = +inf; ymax = -inf;

for k = 1:N
    [~, pts, ~] = FK_G_from_phi1phi2(phi1_traj(k), phi2_traj(k), P);

    for i = 1:numel(names)
        p = pts.(names{i});
        xmin = min(xmin, p(1)); xmax = max(xmax, p(1));
        ymin = min(ymin, p(2)); ymax = max(ymax, p(2));
    end
end

pad = 80;  % increase if you want more whitespace
xL = [xmin-pad, xmax+pad];
yL = [ymin-pad, ymax+pad];

%% ===== plotting setup =====
figure('Name','FK Animation'); clf; hold on; grid on; axis equal;
xlabel('x (mm)'); ylabel('y (mm)');
title('FK animation: \phi_1(t), \phi_2(t) \rightarrow linkage + G');

axis equal
xlim([-500 500])
ylim([-600 200])

trail_on = true;
trail = nan(2,N);

%% ===== animation loop =====
for k = 1:N
    phi1 = phi1_traj(k);
    phi2 = phi2_traj(k);

    phi1_ts = timeseries(phi1_traj*pi/180, t);   % radians
    phi2_ts = timeseries(phi2_traj*pi/180, t);   % radians

    [G, pts, ~] = FK_G_from_phi1phi2(phi1, phi2, P);

    if trail_on
        trail(:,k) = G;
    end

    cla;

    % draw linkage polyline (O-A-B-C-D-E-F-G)
    XY = zeros(2,numel(names));
    for i = 1:numel(names)
        XY(:,i) = pts.(names{i});
    end

    plot(XY(1,:), XY(2,:), '-o', 'LineWidth', 2);

    % labels (comment out if too cluttered)
    for i = 1:numel(names)
        text(XY(1,i), XY(2,i), ['  ' names{i}]);
    end

    % G trail
    if trail_on
        plot(trail(1,1:k), trail(2,1:k), '.', 'MarkerSize', 8);
    end

    % highlight G
    plot(G(1), G(2), 'ro', 'MarkerSize', 9, 'MarkerFaceColor', 'r');

    % show angles
    text(0.02, 0.98, sprintf('\\phi_1 = %.2f°   \\phi_2 = %.2f°', phi1, phi2), ...
        'Units','normalized','VerticalAlignment','top');

    % lock view every frame
    xlim(xL); ylim(yL); axis equal;

    drawnow;
end

%% ==========================================================
%  FK function
%% ==========================================================
function [G, pts, ang] = FK_G_from_phi1phi2(phi1, phi2, P)
    wrap180 = @(a) mod(a + 180.00, 360.00) - 180.00;

    phi1 = wrap180(phi1);
    phi2 = wrap180(phi2);

    L1=P.L1; L2=P.L2; L3=P.L3; L4=P.L4; L5=P.L5; L6=P.L6; L7=P.L7;

    O = [0.00; 0.00];

    A = O + [ L2*cosd(phi2);
             -L2*sind(phi2)];

    a1 = wrap180((phi1 + phi2)/2);
    a3 = wrap180((180.00 - (phi1 + phi2))/2);

    x = sind(a3)*L2/L3;
    x = max(-1, min(1, x));
    a2 = wrap180(asind(x));

    phi4 = wrap180(a3 + phi2 - a2);

    phi3 = wrap180(180.00 - a2 - a3 - phi2);

    B = A + [-L3*cosd(phi3);
             -L3*sind(phi3)];

    C = B + [-L4*cosd(phi4);
             +L4*sind(phi4)];

    a4 = wrap180(180.00 - phi1 - phi4);
    a5 = wrap180(180.00 - a4 + P.k_a5);
    a6 = wrap180(a5 - phi1);

    D = C + [-L5*cosd(a6);
             +L5*sind(a6)];

    E = D + [-L6*cosd(phi1);
             -L6*sind(phi1)];

    F = O + [-L1*cosd(phi1);
             -L1*sind(phi1)];

    phi5 = P.k163 - (180.00 - (phi1 + a6)) - phi1 - P.a7;
    phi5 = wrap180(phi5);

    G = F + [ +L7*cosd(phi5);
              -L7*sind(phi5)];

    pts = struct('O',O,'A',A,'B',B,'C',C,'D',D,'E',E,'F',F,'G',G);
    ang = struct('phi1',phi1,'phi2',phi2,'phi3',phi3,'phi4',phi4,'phi5',phi5, ...
                 'a1',a1,'a2',a2,'a3',a3,'a4',a4,'a5',a5,'a6',a6);
end

