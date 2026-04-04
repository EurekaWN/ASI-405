%% ==========================================================
% Jacobian + Torque mapping from YOUR FK
% - Compute J(q) numerically from FK_G_from_phi1phi2
% - Use tau = J^T * F (virtual work)
% - Includes a Jacobian verification test
%
% Units:
% - FK outputs G in mm (your lengths are mm)
% - Force F in N
% - tau result in N*mm (convert to N*m by /1000)
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

%% ===== Choose a configuration (deg) =====
phi1 = 62.43;
phi2 = 53.22;

%% ===== End-effector force at point G (N) =====
% Make sure Fx,Fy is in the SAME x-y frame as your FK.
Fx = 0;
Fy = 100;  % example: upward/downward depends on your y convention

%% ===== Compute Jacobian and torque =====
[Jdeg, Jrad] = jacobian_numeric_deg(phi1, phi2, P);

F = [Fx; Fy];
tau_Nmm = Jrad.' * F;       % tau = J^T F  (N*mm)
tau_Nm  = tau_Nmm / 1000;   % convert to N*m

% Print results
[G, ~, ~] = FK_G_from_phi1phi2(phi1, phi2, P);

fprintf('--- At phi1=%.3f deg, phi2=%.3f deg ---\n', phi1, phi2);
fprintf('G = (%.3f, %.3f) mm\n\n', G(1), G(2));

disp('J_deg  (mm/deg) = dG/d[phi1,phi2] where phi in degrees:');
disp(Jdeg);

disp('J_rad  (mm/rad) = dG/d[phi1,phi2] where phi in radians:');
disp(Jrad);

fprintf('Force F = [%.3f; %.3f] N\n', Fx, Fy);
fprintf('tau = J^T F = [%.3f; %.3f] N*mm = [%.6f; %.6f] N*m\n\n', ...
    tau_Nmm(1), tau_Nmm(2), tau_Nm(1), tau_Nm(2));

%% ===== Verify Jacobian using velocity test =====
% Pick a joint velocity in rad/s
qd_rad = [0.5; -0.2];    % [phi1dot; phi2dot] rad/s
dt = 1e-3;               % seconds

% Predicted end velocity (mm/s)
Gdot_pred = Jrad * qd_rad;

% Finite difference end velocity from FK (mm/s)
phi1_next = phi1 + (qd_rad(1)*dt) * (180/pi);   % convert rad -> deg step
phi2_next = phi2 + (qd_rad(2)*dt) * (180/pi);

G0 = FK_G_from_phi1phi2(phi1, phi2, P);
G1 = FK_G_from_phi1phi2(phi1_next, phi2_next, P);
Gdot_fd = (G1 - G0) / dt;

fprintf('--- Jacobian verification (velocity) ---\n');
fprintf('Gdot_pred (J*qdot) = [%.3f; %.3f] mm/s\n', Gdot_pred(1), Gdot_pred(2));
fprintf('Gdot_fd   (FD)     = [%.3f; %.3f] mm/s\n', Gdot_fd(1), Gdot_fd(2));
fprintf('error              = [%.3e; %.3e] mm/s\n\n', ...
    Gdot_pred(1)-Gdot_fd(1), Gdot_pred(2)-Gdot_fd(2));

%% ===== Optional: visualize the linkage at this configuration =====
plot_linkage(phi1, phi2, P);

%% ==========================================================
% Part 1: Numerical Jacobian from FK
% - Uses central difference on phi1 and phi2 (in degrees)
% - Converts to radians Jacobian for dynamics/torque mapping
%% ==========================================================
function [Jdeg, Jrad] = jacobian_numeric_deg(phi1_deg, phi2_deg, P)
    % Step size in degrees:
    % too small -> numerical noise; too big -> approximation error.
    h = 1e-3;  % 0.001 deg, try 1e-2 if noisy

    % dG/dphi1 (deg)
    Gp = FK_G_from_phi1phi2(phi1_deg + h, phi2_deg, P);
    Gm = FK_G_from_phi1phi2(phi1_deg - h, phi2_deg, P);
    dG_dphi1_deg = (Gp - Gm) / (2*h);

    % dG/dphi2 (deg)
    Gp = FK_G_from_phi1phi2(phi1_deg, phi2_deg + h, P);
    Gm = FK_G_from_phi1phi2(phi1_deg, phi2_deg - h, P);
    dG_dphi2_deg = (Gp - Gm) / (2*h);

    % Jacobian w.r.t degrees (mm/deg)
    Jdeg = [dG_dphi1_deg, dG_dphi2_deg];

    % Convert to Jacobian w.r.t radians (mm/rad)
    Jrad = Jdeg * (180/pi);
end

%% ==========================================================
% Part 2: Quick plot helper (so you can see the linkage)
%% ==========================================================
function plot_linkage(phi1, phi2, P)
    [~, pts, ~] = FK_G_from_phi1phi2(phi1, phi2, P);

    names = {'O','A','B','C','D','E','F','G'};
    XY = zeros(2,numel(names));
    for i = 1:numel(names)
        XY(:,i) = pts.(names{i});
    end

    figure('Name','Linkage at current pose'); clf; hold on; grid on; axis equal;
    plot(XY(1,:), XY(2,:), '-o', 'LineWidth', 2);
    for i = 1:numel(names)
        text(XY(1,i), XY(2,i), ['  ' names{i}]);
    end
    xlabel('x (mm)'); ylabel('y (mm)');
    title(sprintf('Pose: phi1=%.2f°, phi2=%.2f°', phi1, phi2));
end

%% ==========================================================
% Part 3: YOUR FK function (unchanged)
% Notes:
% - returns G as 2x1 vector [Gx;Gy]
% - pts contains O..G points
%% ==========================================================
function [G, pts, ang] = FK_G_from_phi1phi2(phi1, phi2, P)
    wrap180 = @(a) mod(a + 180.00, 360.00) - 180.00;

    % normalize inputs
    phi1 = wrap180(phi1);
    phi2 = wrap180(phi2);

    % unpack lengths
    L1=P.L1; L2=P.L2; L3=P.L3; L4=P.L4; L5=P.L5; L6=P.L6; L7=P.L7;

    % base
    O = [0.00; 0.00];

    % A
    A = O + [ L2*cosd(phi2);
             -L2*sind(phi2)];

    % angles
    a1 = wrap180((phi1 + phi2)/2);
    a3 = wrap180((180.00 - (phi1 + phi2))/2);

    x = sind(a3)*L2/L3;
    x = max(-1, min(1, x));        % numeric safety
    a2 = wrap180(asind(x));        % branch fixed to asind output

    phi4 = wrap180(a3 + phi2 - a2);

    % B
    phi3 = wrap180(180.00 - a2 - a3 - phi2);

    B = A + [-L3*cosd(phi3);
             -L3*sind(phi3)];

    % C
    C = B + [-L4*cosd(phi4);
             +L4*sind(phi4)];

    % More angles
    a4 = wrap180(180.00 - phi1 - phi4);
    a5 = wrap180(180.00 - a4 + P.k_a5);
    a6 = wrap180(a5 - phi1);

    % D
    D = C + [-L5*cosd(a6);
             +L5*sind(a6)];

    % E
    E = D + [-L6*cosd(phi1);
             -L6*sind(phi1)];

    % F
    F = O + [-L1*cosd(phi1);
             -L1*sind(phi1)];

    % phi5
    phi5 = P.k163 - (180.00 - (phi1 + a6)) - phi1 - P.a7;
    phi5 = wrap180(phi5);

    % G
    G = F + [ +L7*cosd(phi5);
              -L7*sind(phi5)];

    % outputs
    pts = struct('O',O,'A',A,'B',B,'C',C,'D',D,'E',E,'F',F,'G',G);
    ang = struct('phi1',phi1,'phi2',phi2,'phi3',phi3,'phi4',phi4,'phi5',phi5, ...
                 'a1',a1,'a2',a2,'a3',a3,'a4',a4,'a5',a5,'a6',a6);
end