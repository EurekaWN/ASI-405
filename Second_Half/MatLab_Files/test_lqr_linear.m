clear; clc;
load('reduced_model_ABK.mat');

Acl = A - B*K;

tspan = [0 5];

% initial disturbance:
% [theta; dtheta; x; dx; phi; dphi]
x0 = [
    deg2rad(3);   % leg angle error
    0;
    0.02;         % wheel position error
    0;
    deg2rad(5);   % body pitch error
    0
];

[t, x] = ode45(@(t,x) Acl*x, tspan, x0);

% control effort u = -Kx
% These are generalized reduced-model inputs:
%   u(:,1) = T  -> firmware splits across both wheels
%   u(:,2) = Tp -> firmware splits across both legs
u = -(K * x')';   % each row is [T, Tp]

%% ===== state plots =====
figure;
plot(t, rad2deg(x(:,1)), 'LineWidth', 1.5); grid on;
xlabel('Time (s)'); ylabel('\theta (deg)');
title('Leg angle response');

figure;
plot(t, x(:,3), 'LineWidth', 1.5); grid on;
xlabel('Time (s)'); ylabel('x (m)');
title('Wheel position response');

figure;
plot(t, rad2deg(x(:,5)), 'LineWidth', 1.5); grid on;
xlabel('Time (s)'); ylabel('\phi (deg)');
title('Body pitch response');

%% ===== control effort plots =====
figure;
plot(t, u(:,1), 'LineWidth', 1.5); grid on;
xlabel('Time (s)'); ylabel('T');
title('Wheel torque command');

figure;
plot(t, u(:,2), 'LineWidth', 1.5); grid on;
xlabel('Time (s)'); ylabel('Tp');
title('Body / leg torque command');

%% ===== print peak values =====
fprintf('Max abs wheel torque T  = %.4f\n', max(abs(u(:,1))));
fprintf('Max abs body torque Tp  = %.4f\n', max(abs(u(:,2))));
