%% ==========================================================
% Visualize Jacobian (J) at a pose:
% 1) Columns of J as velocity arrows at G
% 2) Velocity ellipse: v = J*qdot, ||qdot||=1
% 3) Torque vs force direction: tau = J^T*F
%% ==========================================================
clear; clc; close all;

%% ===== Parameters (mm, deg) =====
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

%% ===== Pick a configuration =====
phi1 = 62.43;   % deg
phi2 = 53.22;   % deg

%% ===== Compute FK + Jacobian =====
[G, pts, ~] = FK_G_from_phi1phi2(phi1, phi2, P);
[Jdeg, Jrad] = jacobian_numeric_deg(phi1, phi2, P); %#ok<ASGLU>

%% ==========================================================
% FIGURE 1: Linkage + Jacobian column vectors at G
% Interpretation:
% - Column 1 = velocity of G if phi1dot = +1 rad/s, phi2dot = 0
% - Column 2 = velocity of G if phi1dot = 0, phi2dot = +1 rad/s
%% ==========================================================
figure('Name','Jacobian columns as velocity arrows'); clf; hold on; grid on; axis equal;
title('Jacobian columns (velocity at G for unit joint speed)');
xlabel('x (mm)'); ylabel('y (mm)');

% draw linkage
names = {'O','A','B','C','D','E','F','G'};
XY = zeros(2,numel(names));
for i = 1:numel(names)
    XY(:,i) = pts.(names{i});
end
plot(XY(1,:), XY(2,:), '-o', 'LineWidth', 2);
for i = 1:numel(names)
    text(XY(1,i), XY(2,i), ['  ' names{i}]);
end
plot(G(1), G(2), 'ro', 'MarkerFaceColor','r');

% Jacobian columns (mm/s for 1 rad/s)
v1 = Jrad(:,1);   % when qdot=[1;0]
v2 = Jrad(:,2);   % when qdot=[0;1]

% scale arrows so they fit on plot
arrowScale = 0.5; % smaller if too long
quiver(G(1), G(2), arrowScale*v1(1), arrowScale*v1(2), 0, 'LineWidth', 2);
quiver(G(1), G(2), arrowScale*v2(1), arrowScale*v2(2), 0, 'LineWidth', 2);

legend('Linkage','', 'G','J(:,1) (phi1dot=1 rad/s)','J(:,2) (phi2dot=1 rad/s)', ...
    'Location','best');

text(0.02,0.98,sprintf('phi1=%.2f°, phi2=%.2f°',phi1,phi2), ...
    'Units','normalized','VerticalAlignment','top');

%% ==========================================================
% FIGURE 2: Velocity ellipse (manipulability)
% v = J*qdot, with ||qdot|| = 1 rad/s
% This shows all possible end-effector velocities you can generate
% with "unit joint speed" in any direction.
%% ==========================================================
figure('Name','Velocity ellipse'); clf; hold on; grid on; axis equal;
title('Velocity ellipse at G: v = J*qdot, ||qdot||=1');
xlabel('vx (mm/s)'); ylabel('vy (mm/s)');

th = linspace(0,2*pi,400);
qdot = [cos(th); sin(th)];          % ||qdot|| = 1
V = Jrad * qdot;                     % 2xN end velocities

plot(V(1,:), V(2,:), 'LineWidth', 2);
plot(0,0,'k+');

% Also draw the two column vectors as reference axes
quiver(0,0,v1(1),v1(2),0,'LineWidth',2);
quiver(0,0,v2(1),v2(2),0,'LineWidth',2);

legend('Velocity ellipse','origin','J(:,1)','J(:,2)','Location','best');

% manipulability measure (area-ish)
w = abs(det(Jrad)); % mm^2 / (rad^2) per second mapping
text(0.02,0.98,sprintf('|det(J)| = %.2f (bigger = easier to move in 2D)', w), ...
    'Units','normalized','VerticalAlignment','top');

%% ==========================================================
% FIGURE 3: Torque magnitude vs force direction
% tau = J^T * F
% Let |F| = 1 N and rotate its direction.
% This shows which push direction causes huge motor torque (bad near singular).
%% ==========================================================
figure('Name','Torque vs force direction'); clf; hold on; grid on;
title('Torque magnitude vs force direction (|F|=1 N)');
xlabel('force direction angle (deg)'); ylabel('torque magnitude (N·m)');

angF = linspace(-180,180,721);
tauMag = zeros(size(angF));
tau1 = zeros(size(angF));
tau2 = zeros(size(angF));

for k = 1:numel(angF)
    Fx = cosd(angF(k));
    Fy = sind(angF(k));
    tau = Jrad.' * [Fx; Fy];     % N*mm
    tau1(k) = tau(1)/1000;       % N*m
    tau2(k) = tau(2)/1000;
    tauMag(k) = norm(tau)/1000;  % N*m
end

plot(angF, tauMag, 'LineWidth', 2);
plot(angF, abs(tau1), '--', 'LineWidth', 1.5);
plot(angF, abs(tau2), '--', 'LineWidth', 1.5);
legend('||tau||','|tau1|','|tau2|','Location','best');

text(0.02,0.98,'Spikes/large values = near singular direction / bad leverage', ...
    'Units','normalized','VerticalAlignment','top');

%% ==========================================================
% Helper: Numerical Jacobian (degrees -> radians Jacobian)
%% ==========================================================
function [Jdeg, Jrad] = jacobian_numeric_deg(phi1_deg, phi2_deg, P)
    h = 1e-3; % deg

    Gp = FK_G_from_phi1phi2(phi1_deg + h, phi2_deg, P);
    Gm = FK_G_from_phi1phi2(phi1_deg - h, phi2_deg, P);
    dG_dphi1_deg = (Gp - Gm) / (2*h);

    Gp = FK_G_from_phi1phi2(phi1_deg, phi2_deg + h, P);
    Gm = FK_G_from_phi1phi2(phi1_deg, phi2_deg - h, P);
    dG_dphi2_deg = (Gp - Gm) / (2*h);

    Jdeg = [dG_dphi1_deg, dG_dphi2_deg];      % mm/deg
    Jrad = Jdeg * (180/pi);                   % mm/rad
end

%% ==========================================================
% FK function
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
