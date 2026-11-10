function T = drive_torque(omega, p)
%DRIVE_TORQUE Usable torque of one motor at speed omega, power limit included.
%
%   Linear interpolation in the table from build_torque_curve. The grid is
%   uniform, so the index is computed directly.

w  = p.motor.omega_vec;
dw = w(2) - w(1);
x  = min(max(omega, 0), w(end)) / dw;        % position in grid units
i0 = min(floor(x), numel(w) - 2);
f  = x - i0;
T  = (1 - f) .* p.motor.T_vec(i0 + 1) + f .* p.motor.T_vec(i0 + 2);
T  = reshape(T, size(omega));
end
