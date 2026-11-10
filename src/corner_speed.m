function out = corner_speed(radius, p)
%CORNER_SPEED Steady-state corner speed from a single-track (bicycle) model.
%
%   out = corner_speed(radius, p)
%
%   Each axle carries its share of the car's mass around the corner and can
%   supply a limited lateral force, which depends on its load (static weight
%   plus downforce). The axle that saturates first sets the speed:
%
%       a_y,front = 2 * Fy_max(Fz_front) / m_front
%       a_y,rear  = 2 * Fy_max(Fz_rear)  / m_rear
%       a_y       = min(a_y,front , a_y,rear)        v = sqrt(a_y * R)
%
%   Downforce grows with v^2, so the speed is found by iteration.
%
%   out.v      corner speed [m/s]
%   out.ay     lateral acceleration [g]
%   out.limit  'front' (understeer) or 'rear' (oversteer)

veh = p.veh;  aero = p.aero;  g = p.g;
m_f = veh.wd_front * veh.m;
m_r = (1 - veh.wd_front) * veh.m;
v_cap = 80;                               % upper bound for the iteration [m/s]

v = 10;
for it = 1:200
    F_down = 0.5 * aero.rho * aero.ClA * v^2;
    Fz_f = (m_f * g + aero.balance_front     * F_down) / 2;     % per front tyre
    Fz_r = (m_r * g + (1-aero.balance_front) * F_down) / 2;     % per rear tyre
    ay_f = 2 * tire_limit(Fz_f, p.tire, 'y') / m_f;
    ay_r = 2 * tire_limit(Fz_r, p.tire, 'y') / m_r;
    ay   = min(ay_f, ay_r);
    v_new = min(sqrt(ay * radius), v_cap);
    if abs(v_new - v) < 1e-4
        v = v_new;
        break
    end
    v = 0.5 * v + 0.5 * v_new;
end

out.v  = v;
out.ay = ay / g;
if ay_f <= ay_r
    out.limit = 'front';
else
    out.limit = 'rear';
end
end
