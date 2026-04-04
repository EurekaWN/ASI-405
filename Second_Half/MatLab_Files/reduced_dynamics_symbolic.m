function f = reduced_dynamics_symbolic(x, u, p)
%REDUCED_DYNAMICS_SYMBOLIC Reduced wheel-leg model derived from energies.
%
% State:
%   x = [theta; dtheta; xw; dxw; phi; dphi]
%
% Input:
%   u = [T; Tp]
%     T  = generalized wheel torque (shared by both wheels in the reduced model)
%     Tp = generalized body/leg torque at the virtual body-leg joint
%
% Sign conventions used here:
%   theta = leg angle relative to vertical-down.
%           theta = 0 when the axle-to-wheel line is vertical.
%           theta > 0 when the wheel/foot point is forward (Gx > 0).
%   phi   = body pitch relative to upright body pose.
%           phi > 0 means body pitched forward in the reduced model.
%
% Notes:
%   1. This is still a reduced equivalent model, not the full five-bar dynamics.
%   2. The geometry/mass terms should come from your CAD + FK reduction.
%   3. The equations below are derived with Lagrange mechanics rather than
%      hand-written placeholder accelerations, so A/B/K now come from a
%      mechanically consistent reduced model.

theta = x(1);
dtheta = x(2);
xw = x(3);
dxw = x(4);
phi = x(5);
dphi = x(6);

T  = u(1);
Tp = u(2);

g  = p.g;
R  = p.R;
mw = p.mw;
mp = p.mp;
M  = p.M;
L  = p.L;
LM = p.LM;
l  = p.l;
Iw = p.Iw;
Ip = p.Ip;
IM = p.IM;

% Generalized coordinates q = [xw; theta; phi]
q = [xw; theta; phi];
dq = [dxw; dtheta; dphi];

% -------------------------------------------------------------------------
% Equivalent geometry
%
% Wheel center translates along xw.
% Leg COM lies along the virtual leg at distance L from wheel center.
% Body joint lies a further LM along the same virtual leg direction.
% Body COM lies at distance l from the body joint along the body pitch angle.
% -------------------------------------------------------------------------
r_leg = [
    xw + L * sin(theta);
   -L * cos(theta)
];

r_joint = [
    xw + (L + LM) * sin(theta);
   -(L + LM) * cos(theta)
];

r_body = [
    r_joint(1) + l * sin(phi);
    r_joint(2) + l * cos(phi)
];

J_leg = jacobian(r_leg, q);
J_body = jacobian(r_body, q);
v_leg = J_leg * dq;
v_body = J_body * dq;

% -------------------------------------------------------------------------
% Energies
% -------------------------------------------------------------------------
T_energy = ...
    0.5 * (mw + Iw / R^2) * dxw^2 + ...
    0.5 * mp * (v_leg.' * v_leg) + ...
    0.5 * M  * (v_body.' * v_body) + ...
    0.5 * Ip * dtheta^2 + ...
    0.5 * IM * dphi^2;

V_energy = ...
    mp * g * r_leg(2) + ...
    M  * g * r_body(2);

% Generalized forces:
%   xw    gets wheel force T / R
%   theta gets -Tp
%   phi   gets +Tp
Q = [T / R; -Tp; Tp];

% -------------------------------------------------------------------------
% Lagrange equations in manipulator form:
%   M(q) qdd + h(q,dq) = Q
% -------------------------------------------------------------------------
dT_ddq = jacobian(T_energy, dq).';
Mmat = jacobian(dT_ddq, dq);
hvec = jacobian(dT_ddq, q) * dq - jacobian(T_energy, q).' + jacobian(V_energy, q).';
qdd = Mmat \ (Q - hvec);

f = [
    dtheta;
    qdd(2);
    dxw;
    qdd(1);
    dphi;
    qdd(3)
];
end
