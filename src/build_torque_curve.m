function p = build_torque_curve(p)
%BUILD_TORQUE_CURVE Usable torque per motor over speed, as a lookup table.
%
%   Starts from the motor envelope and, where needed, reduces torque until
%   the electrical power of all motors stays inside the tractive system
%   limit:
%
%       n_motors * ( T*omega + losses(T, omega) ) <= P_max
%
%   The limit is electrical, so the losses eat into it: the higher the
%   losses, the less mechanical power is left.
%
%   Adds p.motor.omega_vec [rad/s] (uniform grid) and p.motor.T_vec [Nm].

omega = linspace(0, p.motor.n_max * 1.05, 400);
T     = motor_torque(omega, p.motor);

if p.batt.enable
    P_cap = p.batt.P_max / p.n_motors;          % share of one motor [W]
    for k = 2:numel(omega)
        w  = omega(k);
        Tk = T(k);
        L  = motor_losses(Tk, w, p);
        if Tk * w + L.total > P_cap
            % bisection on torque: electrical power rises monotonically with T
            lo = 0;  hi = Tk;
            for it = 1:40
                mid = 0.5 * (lo + hi);
                L   = motor_losses(mid, w, p);
                if mid * w + L.total > P_cap
                    hi = mid;
                else
                    lo = mid;
                end
            end
            T(k) = lo;
        end
    end
end

p.motor.omega_vec = omega;
p.motor.T_vec     = T;
end
