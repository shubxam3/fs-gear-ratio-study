function res = run_accel(i_gear, p)
%RUN_ACCEL Simulate one standing-start acceleration run for one gear ratio.
%
%   res = run_accel(i_gear, p)
%
%   i_gear  reduction ratio between motor and wheel [-]
%   p       parameter struct, see params_generic
%
%   The car is a longitudinal point mass with four driven wheels (one motor
%   and one planetary gearbox per wheel). At every time step the drive force
%   of each axle is the smaller of what the motors can deliver and what the
%   tyres can transmit.

veh = p.veh;  aero = p.aero;  g = p.g;  dt = p.sim.dt;

% Rotating parts are reflected to the vehicle as additional mass.
m_eff = veh.m + (4*veh.J_wheel + p.n_motors*p.motor.J_rotor*i_gear^2) / veh.r_tire^2;

v = p.sim.v0;  x = 0;  t = 0;  a_prev = 0;

N   = ceil(p.sim.t_max / dt) + 1;
log = zeros(N, 10);
k   = 0;

while x < p.sim.d_target && t < p.sim.t_max

    % Motor speed and usable torque per motor (envelope and power limit)
    omega_mot = v / veh.r_tire * i_gear;
    T_mot     = drive_torque(omega_mot, p);

    % Drive force the two motors of one axle can put on the road
    F_mot_axle = 2 * T_mot * i_gear * p.gear.eta / veh.r_tire;

    % Aero and rolling resistance
    F_down = 0.5 * aero.rho * aero.ClA * v^2;
    F_drag = 0.5 * aero.rho * aero.CdA * v^2;
    F_roll = veh.C_rr * (veh.m * g + F_down);

    % Axle loads. Load transfer uses the acceleration of the previous step,
    % which avoids an algebraic loop (same idea as the Memory block in the
    % Simulink model).
    dFz  = veh.m * a_prev * veh.h_cg / veh.wheelbase;
    Fz_f = max(veh.m*g*veh.wd_front     + F_down*aero.balance_front     - dFz, 0);
    Fz_r =     veh.m*g*(1-veh.wd_front) + F_down*(1-aero.balance_front) + dFz;

    % Each axle delivers the smaller of motor force and tyre limit (two tyres)
    Fx_f = min(F_mot_axle, 2 * tire_limit(Fz_f/2, p.tire, 'x'));
    Fx_r = min(F_mot_axle, 2 * tire_limit(Fz_r/2, p.tire, 'x'));

    a = (Fx_f + Fx_r - F_drag - F_roll) / m_eff;

    k = k + 1;
    log(k, :) = [t, v, x, a, omega_mot*60/(2*pi), T_mot, Fz_f, Fz_r, Fx_f, Fx_r];

    % Semi-implicit Euler step
    v = v + a * dt;
    x = x + v * dt;
    t = t + dt;
    a_prev = a;
end

log = log(1:k, :);
res.t     = log(:,1);   res.v    = log(:,2);   res.x    = log(:,3);
res.a     = log(:,4);   res.n    = log(:,5);   res.T    = log(:,6);
res.Fz_f  = log(:,7);   res.Fz_r = log(:,8);
res.Fx_f  = log(:,9);   res.Fx_r = log(:,10);
res.m_eff = m_eff;

% Losses, battery power and energy from the force actually put on the road
det        = loss_model(res.v, res.Fx_f + res.Fx_r, i_gear, p);
res.P_batt = det.P_batt;
res.E_batt = sum(det.P_batt) * dt + p.loss.P_aux * t;      % [J]
res.detail = det;

res.valid = x >= p.sim.d_target;
if res.valid
    res.t_finish = t;
else
    res.t_finish = NaN;
end
res.v_finish = v * 3.6;                                % [km/h]
res.n_finish = v / veh.r_tire * i_gear * 60/(2*pi);    % [rpm]
end
