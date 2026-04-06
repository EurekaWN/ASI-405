clear; clc;

%% ===== nominal parameters =====
p = struct();

p.g  = 9.81;        % gravity
p.R  = 0.166;       % wheel radius [m], must match firmware WN_WHEEL_RADIUS_M
p.mw =  (259.36 + 151.64*2 + 113.6 + 25.73*4 +15) * 2 / 1000;    % equivalent wheel mass [kg]
p.mp =  (471.98 + 70.62 +  44.28 + 105.83 + 44.98 + 472.93 + 58.26 + 126.12 + 146.57 + 22.03 + 33.58 + 336.4 + 15*12 + 50) * 2 /1000;         % equivalent leg/pole mass [kg]
p.M  = 8.7;         % body mass [kg]

p.L  = 0.11635;        % wheel center to leg COM [m]
p.LM = 0.12769;        % leg COM to body joint equivalent [m]
p.l  = 0.04672;     % body COM offset from body joint / wheel-center ref [m]

p.Iw = 0.01602;        % wheel inertia [kg*m^2]
p.Ip = 0.03468;        % leg inertia [kg*m^2]
p.IM = 0.13421;        % body pitch inertia [kg*m^2]

%% ===== leg-length support feedforward recommendation =====
% The leg-length controller outputs an axial virtual support force along the
% leg. At the nominal upright stance, theta ~= 0 so leg axis ~= world vertical.
%
% Assumptions used here:
% - on_ground: the two legs together support the full robot mass
%              (body + equivalent leg mass + equivalent wheel mass)
% - off_ground: the body/chassis is externally supported, so each leg mainly
%               supports its own equivalent leg + wheel mass
%
% These are feedforward starting points, not final truths. The PD loop still
% trims the residual error around the nominal length.
theta_nom = 0.0;  % upright jig pose
cos_theta_nom = max(cos(theta_nom), 0.2);  % guard against divide-by-small

ff = struct();
ff.total_mass_kg = p.M + p.mp + p.mw;
ff.mass_per_leg_ground_kg = 0.5 * ff.total_mass_kg;
ff.mass_per_leg_air_kg    = 0.5 * (p.mp + p.mw);
ff.force_per_leg_ground_N = ff.mass_per_leg_ground_kg * p.g / cos_theta_nom;
ff.force_per_leg_air_N    = ff.mass_per_leg_air_kg    * p.g / cos_theta_nom;

fprintf('\n');
fprintf('Leg-length feedforward recommendation at theta = %.3f rad\n', theta_nom);
fprintf('  On-ground per leg mass  : %.4f kg\n', ff.mass_per_leg_ground_kg);
fprintf('  On-ground FF per leg    : %.4f N\n', ff.force_per_leg_ground_N);
fprintf('  Off-ground per leg mass : %.4f kg\n', ff.mass_per_leg_air_kg);
fprintf('  Off-ground FF per leg   : %.4f N\n', ff.force_per_leg_air_N);
fprintf('\n');

%% ===== symbolic linearization =====
% reduced_dynamics_symbolic.m defines the reduced-order equations used for
% linearization around the nominal stance.
syms theta dtheta x dx phi dphi T Tp real

xs = [theta; dtheta; x; dx; phi; dphi];
us = [T; Tp];

f = reduced_dynamics_symbolic(xs, us, p);

A_sym = jacobian(f, xs);
B_sym = jacobian(f, us);

%% ===== equilibrium point =====
% upright standing, zero speed, zero torque
x0 = [0; 0; 0; 0; 0; 0];
u0 = [0; 0];

A = double(subs(A_sym, [xs; us], [x0; u0]));

disp('eig(A) = ');
disp(eig(A));

B = double(subs(B_sym, [xs; us], [x0; u0]));

disp('A = ');
disp(A);

disp('B = ');
disp(B);

%% ===== controllability check =====
Co = ctrb(A,B);
fprintf('rank(ctrb) = %d\n', rank(Co));

%% ===== LQR design =====
Q = diag([100, 10, 5, 5, 300, 20]);
R = diag([1, 1]);

K_model = lqr(A,B,Q,R);

% Firmware currently uses forward tilt = negative pitch in g_wn_bal.pitch.
% The reduced model here uses forward tilt = positive phi.
% To export a K that matches firmware directly, flip the phi / dphi columns.
K = K_model;
K(:,5:6) = -K(:,5:6);

% Firmware mapping note:
% u = [T; Tp] here is a single generalized wheel torque and a single
% generalized body/leg torque. The firmware splits T across the 2 wheels
% and splits Tp across the 2 legs before actuator mapping.

Acl_model = A - B*K_model;
disp('eig(A-B*K_model) = ');
disp(eig(Acl_model));

disp('K_model (forward tilt = positive phi) = ');
disp(K_model);

disp('K_firmware (forward tilt = negative pitch in firmware) = ');
disp(K);

save('reduced_model_ABK.mat', 'A', 'B', 'K', 'K_model', 'p', 'ff');
