clear; clc;

% Closed-loop response visualization using the exported LQR gain K.
% This script:
% 1) loads A, B, K from reduced_model_ABK.mat
% 2) simulates x_dot = (A - B*K) x
% 3) plots state and input responses
% 4) saves figures to images_2 for report usage

thisFile = mfilename("fullpath");
thisDir = fileparts(thisFile);
rootDir = fileparts(thisDir);
modelFile = fullfile(thisDir, "reduced_model_ABK.mat");

if ~isfile(modelFile)
    fprintf("reduced_model_ABK.mat not found. Running build_reduced_model.m ...\n");
    run(fullfile(thisDir, "build_reduced_model.m"));
    % build_reduced_model.m starts with "clear", so recover local paths.
    thisFile = mfilename("fullpath");
    thisDir = fileparts(thisFile);
    rootDir = fileparts(thisDir);
    modelFile = fullfile(thisDir, "reduced_model_ABK.mat");
end

S = load(modelFile);   % expects A, B, K, K_model
A = S.A;
B = S.B;
pitchSignMap = diag([1 1 1 1 -1 -1]);  % x_fw = pitchSignMap * x_model

hasKFirmware = isfield(S, "K");
hasKModel = isfield(S, "K_model");

if hasKFirmware
    K_firmware = S.K;  % exported for firmware convention (pitch is negative forward)
    % Equivalent model-space gain for x_model=[..., +phi, +dphi].
    K_sim = K_firmware * pitchSignMap;
    kLabel = "K_firmware (mapped to model state sign)";
elseif hasKModel
    K_sim = S.K_model;
    kLabel = "K_model";
else
    error("Neither K nor K_model found in reduced_model_ABK.mat");
end

Acl = A - B*K_sim;

fprintf("\nUsing gain for simulation: %s\n", kLabel);
fprintf("Closed-loop eigenvalues (A-BK):\n");
disp(eig(Acl));

%% ===== simulation setup =====
tEnd = 6.0;
tspan = [0, tEnd];

% Initial disturbance:
% x = [theta; dtheta; x; dx; phi; dphi]
x0 = [
    deg2rad(3.0);   % theta (rad)
    0.0;            % dtheta (rad/s)
    0.020;          % x (m)
    0.0;            % dx (m/s)
    deg2rad(5.0);   % phi (rad)
    0.0             % dphi (rad/s)
];

[t, x] = ode45(@(~, xk) Acl*xk, tspan, x0);
if hasKFirmware
    x_fw = (pitchSignMap * x')';
    u = -(K_firmware * x_fw')';  % firmware-side formulation
else
    u = -(K_sim * x')';
end

%% ===== figure 1: all state responses =====
fig1 = figure('Color', 'w', 'Name', 'LQR Closed-Loop State Response');
tiledlayout(3, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile;
plot(t, rad2deg(x(:,1)), 'LineWidth', 1.6); grid on;
xlabel('Time (s)'); ylabel('theta (deg)');
title('Leg Angle theta');

nexttile;
plot(t, rad2deg(x(:,2)), 'LineWidth', 1.6); grid on;
xlabel('Time (s)'); ylabel('theta_dot (deg/s)');
title('Leg Angular Rate theta_dot');

nexttile;
plot(t, x(:,3), 'LineWidth', 1.6); grid on;
xlabel('Time (s)'); ylabel('x (m)');
title('Wheel Position x');

nexttile;
plot(t, x(:,4), 'LineWidth', 1.6); grid on;
xlabel('Time (s)'); ylabel('x_dot (m/s)');
title('Wheel Speed x_dot');

nexttile;
plot(t, rad2deg(x(:,5)), 'LineWidth', 1.6); grid on;
xlabel('Time (s)'); ylabel('phi (deg)');
title('Body Pitch phi');

nexttile;
plot(t, rad2deg(x(:,6)), 'LineWidth', 1.6); grid on;
xlabel('Time (s)'); ylabel('phi_dot (deg/s)');
title('Body Pitch Rate phi_dot');

sgtitle(sprintf('Closed-Loop State Response (%s)', kLabel));

%% ===== figure 2: control response =====
fig2 = figure('Color', 'w', 'Name', 'LQR Control Effort');
tiledlayout(2, 1, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile;
plot(t, u(:,1), 'LineWidth', 1.7); grid on;
xlabel('Time (s)'); ylabel('T');
title('Generalized Wheel Torque Command T');

nexttile;
plot(t, u(:,2), 'LineWidth', 1.7); grid on;
xlabel('Time (s)'); ylabel('Tp');
title('Generalized Body/Leg Torque Command Tp');

sgtitle('Control Input Response from u = -Kx');

%% ===== figure 3: all states in one plot (normalized) =====
% To place states with different units into one graph, each state is
% normalized by its own peak absolute value.
allSignals = [
    rad2deg(x(:,1)), ... % theta
    rad2deg(x(:,2)), ... % dtheta
    x(:,3), ...          % x
    x(:,4), ...          % dx
    rad2deg(x(:,5)), ... % phi
    rad2deg(x(:,6))      % dphi
];

sigNames = {
    'theta (deg)',
    'theta\_dot (deg/s)',
    'x (m)',
    'x\_dot (m/s)',
    'phi (deg)',
    'phi\_dot (deg/s)'
};

peakVals = max(abs(allSignals), [], 1);
peakVals(peakVals < 1e-12) = 1.0;
allSignalsNorm = allSignals ./ peakVals;

fig3 = figure('Color', 'w', 'Name', 'LQR Combined Response');
ax = axes(fig3); hold(ax, 'on'); grid(ax, 'on');
colorSet = lines(size(allSignalsNorm, 2));
for k = 1:size(allSignalsNorm, 2)
    plot(t, allSignalsNorm(:,k), 'LineWidth', 1.7, 'Color', colorSet(k,:));
end
xlabel('Time (s)');
ylabel('Normalized response (signal / peak)');
title('Combined LQR Response (All Signals, Different Colors)');
legend(sigNames, 'Location', 'eastoutside');

%% ===== figure 4: animated line growth over time =====
fig4 = figure('Color', 'w', 'Name', 'LQR Combined Response Animated');
fig4.Position = [120, 80, 1220, 720];
ax4 = axes(fig4); hold(ax4, 'on'); grid(ax4, 'on');
if isprop(ax4, 'Toolbar') && ~isempty(ax4.Toolbar)
    ax4.Toolbar.Visible = 'off';
end
xlabel('Time (s)');
ylabel('Normalized response (signal / peak)');
title('Combined LQR Response Animation (Line Extending Over Time)');

xlim(ax4, [t(1), t(end)]);
yMin = min(allSignalsNorm(:));
yMax = max(allSignalsNorm(:));
yPad = 0.08 * max(1e-3, yMax - yMin);
ylim(ax4, [yMin - yPad, yMax + yPad]);

nSig = size(allSignalsNorm, 2);
hAnim = gobjects(nSig, 1);
for k = 1:nSig
    hAnim(k) = plot(ax4, nan, nan, 'LineWidth', 1.9, 'Color', colorSet(k,:));
end
legend(ax4, sigNames, 'Location', 'eastoutside');

%% ===== console summary =====
fprintf("\nPeak response summary:\n");
fprintf("  max |theta|      = %.3f deg\n", max(abs(rad2deg(x(:,1)))));
fprintf("  max |x|          = %.4f m\n", max(abs(x(:,3))));
fprintf("  max |phi|        = %.3f deg\n", max(abs(rad2deg(x(:,5)))));
fprintf("  max |T|          = %.4f\n", max(abs(u(:,1))));
fprintf("  max |Tp|         = %.4f\n", max(abs(u(:,2))));

%% ===== save images for report =====
imgDir = fullfile(rootDir, "images_2");
if exist(imgDir, "dir")
    out1 = fullfile(imgDir, 'LQR_Kmatrix_State_Response.png');
    out2 = fullfile(imgDir, 'LQR_Kmatrix_Control_Response.png');
    out3 = fullfile(imgDir, 'LQR_Kmatrix_All_In_One.png');
    out5 = fullfile(imgDir, 'LQR_Kmatrix_All_In_One_Animated.gif');
    exportgraphics(fig1, out1, 'Resolution', 220);
    exportgraphics(fig2, out2, 'Resolution', 220);
    exportgraphics(fig3, out3, 'Resolution', 220);

    % Animated export (line extends as time passes)
    vidDir = fullfile(rootDir, "videos_2");
    if ~exist(vidDir, "dir")
        mkdir(vidDir);
    end
    out4 = fullfile(vidDir, 'LQR_Kmatrix_All_In_One_Animated.mp4');

    % Use a uniform animation timeline instead of raw ode45 time points,
    % otherwise adaptive solver output can produce only a handful of frames.
    targetFrames = 120;
    playbackSeconds = 8;
    frameRate = round(targetFrames / playbackSeconds);
    delayTime = playbackSeconds / targetFrames;
    tAnim = linspace(t(1), t(end), targetFrames);
    allSignalsAnim = interp1(t, allSignalsNorm, tAnim, 'pchip');

    try
        if isfile(out5)
            delete(out5);
        end

        [out4Folder, out4Name, out4Ext] = fileparts(out4);
        out4Try = out4;
        if isfile(out4Try)
            stamp = datestr(now, 'yyyymmdd_HHMMSS');
            out4Try = fullfile(out4Folder, sprintf('%s_%s%s', out4Name, stamp, out4Ext));
        end

        vw = VideoWriter(out4Try, 'MPEG-4');
        vw.FrameRate = frameRate;
        open(vw);
        tmpFramePng = fullfile(tempdir, 'lqr_kmatrix_anim_frame.png');

        for ii = 1:targetFrames
            for k = 1:nSig
                set(hAnim(k), 'XData', tAnim(1:ii), 'YData', allSignalsAnim(1:ii, k));
            end
            drawnow;

            exportgraphics(fig4, tmpFramePng, 'Resolution', 180);
            rgb = imread(tmpFramePng);
            fr = im2frame(rgb);
            writeVideo(vw, fr);

            [im, map] = rgb2ind(rgb, 256);
            if ii == 1
                imwrite(im, map, out5, 'gif', 'LoopCount', inf, 'DelayTime', delayTime);
            else
                imwrite(im, map, out5, 'gif', 'WriteMode', 'append', 'DelayTime', delayTime);
            end
        end
        close(vw);
        out4 = out4Try;
        if exist('tmpFramePng', 'var') && isfile(tmpFramePng)
            delete(tmpFramePng);
        end
    catch animErr
        if exist('vw', 'var')
            try
                close(vw);
            catch
            end
        end
        if exist('tmpFramePng', 'var') && isfile(tmpFramePng)
            delete(tmpFramePng);
        end
        warning('Animation export failed: %s', animErr.message);
    end

    fprintf("\nSaved figures/videos:\n  %s\n  %s\n  %s\n  %s\n  %s\n", out1, out2, out3, out4, out5);
else
    fprintf("\nimages_2 folder not found. Figures not auto-saved.\n");
end
