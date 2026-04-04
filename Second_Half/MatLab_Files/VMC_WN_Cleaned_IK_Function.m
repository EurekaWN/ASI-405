%% ==========================================================
%  One-file Inverse Kinematics demo (uses YOUR FK equations)
%  - Put this entire file as: IK_FK_OneFile.m
%  - Press Run
%
%  Requires: Optimization Toolbox (lsqnonlin).
%  If you don't have it, tell me and I'll swap to fminsearch.
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

P.k_a5 = 16.44;   % deg
P.a7   = 21.21;   % deg
P.k163 = 163.51;  % deg

%% ===== Target G (edit this) =====
G_des = [41.56; -292.4];   % [Gx; Gy]

%% ===== Joint limits (edit to your real limits) =====
bounds.lb = [0; 0];         % [phi1_min; phi2_min] deg
bounds.ub = [140; 140];     % [phi1_max; phi2_max] deg

%% ===== Initial guess (important!) =====
phi_init = [60; 50];        % [phi1; phi2] deg

%% ===== Solve IK (single start) =====
[phi_sol, info] = IK_phi1phi2_from_G(G_des, P, phi_init, bounds);

fprintf('--- IK result (single start) ---\n');
fprintf('phi1 = %.6f deg\n', phi_sol(1));
fprintf('phi2 = %.6f deg\n', phi_sol(2));
fprintf('G_sol = (%.6f, %.6f)\n', info.G_sol(1), info.G_sol(2));
fprintf('G_err = (%.6e, %.6e), |err|=%.6e\n\n', info.G_err(1), info.G_err(2), norm(info.G_err));

%% ===== Multi-start (more robust) =====
[best_phi, best_info] = IK_multistart(G_des, P, bounds);

fprintf('--- IK result (multi-start best) ---\n');
fprintf('phi1 = %.6f deg\n', best_phi(1));
fprintf('phi2 = %.6f deg\n', best_phi(2));
fprintf('G_sol = (%.6f, %.6f)\n', best_info.G_sol(1), best_info.G_sol(2));
fprintf('G_err = (%.6e, %.6e), |err|=%.6e\n\n', best_info.G_err(1), best_info.G_err(2), norm(best_info.G_err));

%% ===== Plot the solved configuration =====
[G, pts] = FK_only(best_phi(1), best_phi(2), P);

names = {'O','A','B','C','D','E','F','G'};
XY = zeros(2,numel(names));
for i = 1:numel(names)
    XY(:,i) = pts.(names{i});
end

figure; clf; hold on; axis equal; grid on;
plot(XY(1,:), XY(2,:), '-o', 'LineWidth', 2);
plot(G_des(1), G_des(2), 'rx', 'MarkerSize', 12, 'LineWidth', 2);
text(G_des(1), G_des(2), '  G_{des}');
for i = 1:numel(names)
    text(XY(1,i), XY(2,i), ['  ' names{i}]);
end
xlabel('x (mm)'); ylabel('y (mm)');
title('Solved IK configuration (blue) and desired G (red x)');

%% ==========================================================
%  LOCAL FUNCTIONS (everything in this one file)
%% ==========================================================

function [phi_sol, info] = IK_phi1phi2_from_G(G_des, P, phi_init, bounds)
    if size(G_des,2) > 1, G_des = G_des(:); end
    wrap180 = @(a) mod(a + 180.0, 360.0) - 180.0;

    x0 = phi_init(:);
    x0 = max(bounds.lb, min(bounds.ub, x0));

    fun = @(x) fk_error(x, G_des, P);

    opts = optimoptions('lsqnonlin', ...
        'Display','off', ...
        'MaxFunctionEvaluations', 8000, ...
        'MaxIterations', 300, ...
        'FunctionTolerance', 1e-12, ...
        'StepTolerance', 1e-12);

    [x, resnorm, residual, exitflag, output] = lsqnonlin(fun, x0, bounds.lb, bounds.ub, opts);

    x = wrap180(x);
    phi_sol = x(:);

    info = struct();
    info.resnorm  = resnorm;
    info.residual = residual;
    info.exitflag = exitflag;
    info.output   = output;

    [G_sol, ~] = FK_only(phi_sol(1), phi_sol(2), P);
    info.G_sol = G_sol;
    info.G_err = G_sol - G_des;
end

function e = fk_error(x, G_des, P)
    phi1 = x(1); phi2 = x(2);
    [G, ~] = FK_only(phi1, phi2, P);
    e = G - G_des;
end

function [best_phi, best_info] = IK_multistart(G_des, P, bounds)
    best_cost = inf;
    best_phi  = [NaN; NaN];
    best_info = struct();

    phi1_guesses = linspace(bounds.lb(1), bounds.ub(1), 7);
    phi2_guesses = linspace(bounds.lb(2), bounds.ub(2), 7);

    for p1 = phi1_guesses
        for p2 = phi2_guesses
            try
                [phi, info] = IK_phi1phi2_from_G(G_des, P, [p1; p2], bounds);
                cost = norm(info.G_err);
                if cost < best_cost
                    best_cost = cost;
                    best_phi  = phi;
                    best_info = info;
                end
            catch
                % ignore failures
            end
        end
    end
end

function [G, pts] = FK_only(phi1, phi2, P)

  %Forward kinematics equations packaged as a function
    wrap180 = @(a) mod(a + 180.00, 360.00) - 180.00;

    phi1 = wrap180(phi1);
    phi2 = wrap180(phi2);

    L1=P.L1; L2=P.L2; L3=P.L3; L4=P.L4; L5=P.L5; L6=P.L6; L7=P.L7;

    Ox = 0.00; Oy = 0.00;
    O = [Ox; Oy];

    % A
    Ax = Ox + L2 * cosd(phi2);
    Ay = Oy - L2 * sind(phi2);
    A  = [Ax; Ay];

    % Angles
    a1 = wrap180( (phi1 + phi2) / 2 );
    a3 = wrap180( (180.00 - (phi1 + phi2)) / 2 );

    x = sind(a3) * L2 / L3;
    x = max(-1, min(1, x));
    a2 = wrap180( asind(x) );

    phi4 = wrap180( a3 + phi2 - a2 );

    % B
    phi3 = wrap180( 180.00 - a2 - a3 - phi2 );

    Bx = Ax - L3 * cosd(phi3);
    By = Ay - L3 * sind(phi3);
    B  = [Bx; By];

    % C
    Cx = Bx - L4 * cosd(phi4);
    Cy = By + L4 * sind(phi4);
    C  = [Cx; Cy];

    % More angles
    a4 = wrap180( 180.00 - phi1 - phi4 );
    a5 = wrap180( 180.00 - a4 + P.k_a5 );
    a6 = wrap180( a5 - phi1 );

    % D
    Dx = Cx - L5 * cosd(a6);
    Dy = Cy + L5 * sind(a6);
    D  = [Dx; Dy];

    % E
    Ex = Dx - L6 * cosd(phi1);
    Ey = Dy - L6 * sind(phi1);
    E  = [Ex; Ey];

    % F
    Fx = Ox - L1 * cosd(phi1);
    Fy = Oy - L1 * sind(phi1);
    F  = [Fx; Fy];

    % phi5 and G
    phi5 = P.k163 - (180.00 - (phi1 + a6)) - phi1 - P.a7;
    phi5 = wrap180(phi5);

    Gx = Fx + L7 * cosd(phi5);
    Gy = Fy - L7 * sind(phi5);
    G  = [Gx; Gy];

    pts = struct('O',O,'A',A,'B',B,'C',C,'D',D,'E',E,'F',F,'G',G);
end