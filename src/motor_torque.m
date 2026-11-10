function T = motor_torque(omega, motor)
%MOTOR_TORQUE Peak torque of one motor as a function of speed.
%
%   Piecewise curve read from the datasheet:
%     omega <= n_base           constant torque  T_peak
%     n_base < omega <= n_max   constant power   P_peak / omega
%     omega >  n_max            no torque
%
%   omega  motor speed [rad/s], scalar or array
%   T      torque [Nm]

T = (omega <= motor.n_base) .* motor.T_peak ...
  + (omega >  motor.n_base & omega <= motor.n_max) .* (motor.P_peak ./ max(omega, 1e-6));
end
