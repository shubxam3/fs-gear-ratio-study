function out = loss_model(v, F_drive, i_gear, p)
%LOSS_MODEL Drivetrain losses and battery power along a speed profile.
%
%   v        vehicle speed [m/s], vector
%   F_drive  total force at the tyres [N], negative when braking
%   i_gear   gear ratio [-]
%
%   Power flows battery -> inverter -> motor -> gearbox -> wheel when
%   driving and the other way when braking. Output fields are vectors:
%     n_rpm, T_motor              motor speed and torque (per motor)
%     P_cu, P_core, P_inv, P_gb   losses of all motors together [W]
%     P_batt                      battery power [W], negative when recovering

v = v(:);  F_drive = F_drive(:);
N = p.n_motors;  r = p.veh.r_tire;  eta = p.gear.eta;

omega     = v ./ r * i_gear;
out.n_rpm = omega * 60/(2*pi);

driving = F_drive >= 0;

% Torque per motor: the gearbox loss is paid by the motor when driving
% and by the wheel when braking.
T_motor           = F_drive .* r ./ (N * i_gear);
T_motor(driving)  = T_motor(driving)  ./ eta;
T_motor(~driving) = T_motor(~driving) .* eta;
out.T_motor = T_motor;

L = motor_losses(T_motor, omega, p);
out.P_cu   = N * L.cu;
out.P_core = N * L.core;
out.P_inv  = N * L.inv;

P_wheel  = F_drive .* v;
P_shaft  = N * T_motor .* omega;
out.P_gb = abs(P_wheel - P_shaft);

P_batt           = P_shaft + out.P_cu + out.P_core + out.P_inv;
P_batt(~driving) = P_batt(~driving) .* p.loss.regen;
out.P_batt = P_batt;
out.P_loss = out.P_cu + out.P_core + out.P_inv + out.P_gb;
end
