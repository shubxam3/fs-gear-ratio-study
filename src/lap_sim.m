function R = lap_sim(i_gear, trk, p)
%LAP_SIM Quasi-steady-state lap time and energy for one gear ratio.
%
%   R = lap_sim(i_gear, trk, p)
%
%   Three passes over the track:
%     1. corner pass    speed limit at every point from lateral grip
%     2. forward pass   accelerate from each point, limited by motors and tyres
%     3. backward pass  brake into each point, limited by tyres
%   The speed profile is the minimum of the three. Losses and battery energy
%   are then evaluated along that profile.
%
%   R.t_total  event time [s]           R.E_batt  battery energy [J]
%   R.v        speed profile [m/s]      R.detail  loss breakdown (loss_model)

veh = p.veh;  aero = p.aero;  g = p.g;
s = trk.s;  kappa = abs(trk.kappa);  ds = trk.ds;  n = numel(s);
r = veh.r_tire;  wd = veh.wd_front;  bal = aero.balance_front;
hL = veh.h_cg / veh.wheelbase;

m     = veh.m;                                                         % for loads
m_eff = m + (4*veh.J_wheel + p.n_motors*p.motor.J_rotor*i_gear^2) / r^2;  % for inertia

v_max = p.motor.n_max * r / i_gear;       % motor speed limit

% --- 1) corner limit ---
v_corner = v_max * ones(n, 1);
radii = unique(1 ./ kappa(kappa > 1e-6));
for q = 1:numel(radii)
    c = corner_speed(radii(q), p);
    v_corner(abs(kappa - 1/radii(q)) < 1e-9) = min(c.v, v_max);
end

% --- 2) forward pass ---
v_fwd = v_corner;
if trk.standing
    v_fwd(1) = 0.5;
    passes = 1;
else
    passes = 2;                           % flying lap: second pass carries speed over the line
end
for pass = 1:passes
    for k = 1:n-1
        v = v_fwd(k);
        F_down = 0.5 * aero.rho * aero.ClA * v^2;
        F_res  = 0.5 * aero.rho * aero.CdA * v^2 + veh.C_rr * (m*g + F_down);
        F_axle = 2 * drive_torque(v / r * i_gear, p) * i_gear * p.gear.eta / r;
        a = 0;
        for it = 1:4                      % load transfer depends on a
            dFz  = m * a * hL;
            Fz_f = max(wd*m*g     + bal*F_down     - dFz, 0) / 2;
            Fz_r = max((1-wd)*m*g + (1-bal)*F_down + dFz, 0) / 2;
            F_t  = min(F_axle, 2 * tire_limit(Fz_f, p.tire, 'x')) ...
                 + min(F_axle, 2 * tire_limit(Fz_r, p.tire, 'x'));
            a    = max(F_t - F_res, 0) / m_eff;
        end
        v_fwd(k+1) = min(v_corner(k+1), sqrt(v^2 + 2*a*ds));
    end
    if pass < passes
        v_fwd(1) = min(v_corner(1), v_fwd(end));
    end
end

% --- 3) backward pass ---
v_bwd = v_corner;
for pass = 1:passes
    for k = n-1:-1:1
        v = v_bwd(k+1);
        F_down = 0.5 * aero.rho * aero.ClA * v^2;
        F_res  = 0.5 * aero.rho * aero.CdA * v^2 + veh.C_rr * (m*g + F_down);
        a = 0;
        for it = 1:3                      % braking loads the front axle
            dFz  = m * a * hL;
            Fz_f = max(wd*m*g     + bal*F_down     + dFz, 0) / 2;
            Fz_r = max((1-wd)*m*g + (1-bal)*F_down - dFz, 0) / 2;
            F_b  = 2 * tire_limit(Fz_f, p.tire, 'x') + 2 * tire_limit(Fz_r, p.tire, 'x');
            a    = (F_b + F_res) / m_eff;
        end
        v_bwd(k) = min(v_bwd(k), sqrt(v^2 + 2*a*ds));
    end
    if pass < passes
        v_bwd(end) = min(v_corner(end), v_bwd(1));
    end
end

% --- speed profile, time ---
v = max(min([v_corner, v_fwd, v_bwd], [], 2), 0.5);
v_mid = 0.5 * (v(1:end-1) + v(2:end));
dt    = ds ./ v_mid;
t_lap = sum(dt);

% --- force at the tyres, losses, energy ---
a_long  = (v(2:end).^2 - v(1:end-1).^2) / (2*ds);
a_long  = [a_long; a_long(end)];
F_down  = 0.5 * aero.rho * aero.ClA * v.^2;
F_drive = m_eff * a_long + 0.5 * aero.rho * aero.CdA * v.^2 + veh.C_rr * (m*g + F_down);

det = loss_model(v, F_drive, i_gear, p);
P_b = 0.5 * (det.P_batt(1:end-1) + det.P_batt(2:end)) + p.loss.P_aux;

R.t_lap    = t_lap;
R.t_total  = t_lap * trk.laps;
R.E_batt   = sum(P_b .* dt) * trk.laps;
R.v        = v;
R.s        = s;
R.F_drive  = F_drive;
R.detail   = det;
R.n_max    = max(det.n_rpm);
end
