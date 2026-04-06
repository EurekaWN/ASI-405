%% Gear ratio effect on speed and acceleration (M3508-based)
% This script reproduces the "rated speed vs acceleration" trend used for
% selecting the big inner gearing ratio.
%
% Source workflow:
% 1) Extract motor curve points from motor spec plot using WebPlotDigitizer.
% 2) Fit/interpolate rated motor speed at selected torque.
% 3) Sweep motor:wheel gear ratio and compute:
%    - Wheel rated linear speed
%    - Ideal acceleration limited by traction ceiling
% 4) Mark the practical chosen ratio (3.76).

clear; clc; close all;

%% 1) Motor-curve points (replace with exact extracted data if updated)
% Torque [N*m] vs motor speed [rpm], digitized from M3508 characteristic plot.
torque_pts = [0.0 0.5 1.0 2.0 3.0 4.0 4.5 4.8 5.0];
rpm_pts    = [520 510 500 490 475 455 430 380 320];

%% 2) Design parameters
rated_torque_nm = 3.0;      % Operating torque used for rated-speed estimate
wheel_diameter_m = 0.4064;  % 16-inch wheel
wheel_radius_m = wheel_diameter_m/2;

mass_robot_kg = 25;         % Total robot mass
driven_wheels = 2;          % Left + right wheel channels
drive_eff = 0.90;           % Combined transmission efficiency estimate
mu_eff = 0.68;              % Effective traction coefficient for acceleration cap
g = 9.81;

ratio_vec = linspace(2.0, 10.0, 161);  % motor : wheel
chosen_ratio = 3.76;

%% 3) Rated speed at selected torque
rated_motor_rpm = interp1(torque_pts, rpm_pts, rated_torque_nm, 'pchip', 'extrap');

% Wheel linear speed at each ratio
wheel_rpm = rated_motor_rpm ./ ratio_vec;
rated_speed_mps = wheel_rpm .* (2*pi*wheel_radius_m/60);

% Acceleration estimate from wheel force
wheel_torque_nm = rated_torque_nm .* ratio_vec .* drive_eff;
drive_force_n = driven_wheels .* (wheel_torque_nm ./ wheel_radius_m);
accel_raw = drive_force_n ./ mass_robot_kg;

% Traction-limited acceleration
accel_limit = mu_eff * g;
accel_mps2 = min(accel_raw, accel_limit);

%% 4) Plot
figure('Color','w','Position',[120 120 980 520]);
yyaxis left;
plot(ratio_vec, rated_speed_mps, 'b-', 'LineWidth', 2); hold on;
ylabel('Rated Speed (m/s)');
ylim([max(0,min(rated_speed_mps)-0.15), max(rated_speed_mps)+0.2]);

yyaxis right;
plot(ratio_vec, accel_mps2, '--', 'Color', [0.85 0.33 0.10], 'LineWidth', 2);
ylabel('Acceleration (m/s^2)');
ylim([max(0,min(accel_mps2)-0.3), max(accel_mps2)+0.4]);

xlabel('Gear ratio (motor : wheel)');
title('Balancing Robot - Rated Speed and Acceleration vs Gear Ratio');
grid on;

% Mark chosen ratio
yyaxis left;
speed_chosen = interp1(ratio_vec, rated_speed_mps, chosen_ratio, 'linear');
plot(chosen_ratio, speed_chosen, 'o', 'MarkerSize', 8, ...
    'MarkerFaceColor', 'w', 'MarkerEdgeColor', 'b', 'LineWidth', 1.5);

yyaxis right;
accel_chosen = interp1(ratio_vec, accel_mps2, chosen_ratio, 'linear');
plot(chosen_ratio, accel_chosen, 'o', 'MarkerSize', 8, ...
    'MarkerFaceColor', 'w', 'MarkerEdgeColor', [0.85 0.33 0.10], 'LineWidth', 1.5);

legend({'Rated speed at 3 Nm', 'Acceleration', 'Chosen ratio'}, ...
    'Location', 'northeast');

%% 5) Output key points
fprintf('Rated motor speed at %.2f Nm: %.2f rpm\\n', rated_torque_nm, rated_motor_rpm);
fprintf('Chosen ratio: %.2f\\n', chosen_ratio);
fprintf('Rated wheel speed at chosen ratio: %.4f m/s\\n', speed_chosen);
fprintf('Estimated acceleration at chosen ratio: %.4f m/s^2\\n', accel_chosen);
fprintf('Acceleration limit used: %.4f m/s^2\\n', accel_limit);

% Optional table for export
T = table(ratio_vec(:), rated_speed_mps(:), accel_mps2(:), ...
    'VariableNames', {'GearRatio_MotorToWheel', 'RatedSpeed_mps', 'Acceleration_mps2'});
disp(T(1:10:end,:));
