function L = motor_losses(T, omega, p)
%MOTOR_LOSSES Electrical losses of one motor and its inverter.
%
%   T      motor torque [Nm], sign ignored (losses are the same when braking)
%   omega  motor speed [rad/s]
%
%   L.cu    copper loss    3 * R(T_wind) * I^2,  I = |T| / kt
%   L.core  core loss      a*n + b*n^2,          n in 1000 rpm
%   L.inv   inverter loss  switching (k_sw * I) + conduction (3 * R_on * I^2)
%   L.total sum of the three [W]

I = abs(T) ./ p.motor.kt;                                      % phase current [A]

R = p.motor.R_phase * (1 + 0.00393 * (p.motor.T_wind - 20));   % copper at T_wind
L.cu = 3 * R .* I.^2;

n_k = abs(omega) * 60/(2*pi) / 1000;                           % speed in 1000 rpm
L.core = p.loss.core_a .* n_k + p.loss.core_b .* n_k.^2;

L.inv = p.loss.inv_k_sw .* I + 3 * p.loss.inv_R_on .* I.^2;

L.total = L.cu + L.core + L.inv;
end
