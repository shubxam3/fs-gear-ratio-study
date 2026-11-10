function p = params_generic()
%PARAMS_GENERIC Parameter set for the gear ratio study.
%
%   The motor values are taken from the public Fischer TI085-052-070
%   datasheet. All vehicle, aero, tyre and loss values are representative
%   Formula Student figures chosen for this public repository. They are
%   NOT the parameters of any particular team's car, and the loss
%   coefficients are illustrative, not measured.

% --- Motor (one per wheel, four in total) ---
p.n_motors      = 4;
p.motor.T_peak  = 29.1;                 % peak torque [Nm]
p.motor.P_peak  = 35366;                % peak power [W]
p.motor.n_base  = 11600 * 2*pi/60;      % end of constant-torque region [rad/s]
p.motor.n_max   = 20000 * 2*pi/60;      % maximum speed [rad/s]
p.motor.J_rotor = 0.33e-3;              % rotor inertia [kg m^2]
p.motor.kt      = 0.492;                % torque constant [Nm/A]
p.motor.R_phase = 0.126;                % phase resistance at 20 degC [ohm]
p.motor.T_wind  = 40;                   % winding temperature [degC]
                                        % (about 40 for one sprint, 100 in endurance)

% --- Losses ---
p.loss.core_a   = 18;                   % core loss, linear term [W per 1000 rpm]
p.loss.core_b   = 0.30;                 % core loss, quadratic term [W per (1000 rpm)^2]
p.loss.inv_k_sw = 2.0;                  % inverter switching loss [W per A]
p.loss.inv_R_on = 0.012;                % inverter on-resistance per switch [ohm]
p.loss.P_aux    = 300;                  % pumps, fans, electronics [W]
p.loss.regen    = 0.90;                 % share of recovered energy that is counted [-]

% --- Gearbox (planetary, one per wheel) ---
p.gear.eta      = 0.95;                 % efficiency [-]

% --- Vehicle ---
p.veh.m         = 250;                  % mass including driver [kg]
p.veh.r_tire    = 0.228;                % loaded tyre radius [m]
p.veh.h_cg      = 0.30;                 % centre of gravity height [m]
p.veh.wheelbase = 1.55;                 % [m]
p.veh.wd_front  = 0.50;                 % static front weight fraction [-]
p.veh.J_wheel   = 0.30;                 % wheel assembly inertia, per wheel [kg m^2]
p.veh.C_rr      = 0.015;                % rolling resistance coefficient [-]

% --- Aerodynamics ---
p.aero.rho           = 1.18;            % air density [kg/m^3]
p.aero.CdA           = 1.30;            % drag area [m^2]
p.aero.ClA           = 3.00;            % downforce area [m^2]
p.aero.balance_front = 0.50;            % share of downforce on the front axle [-]

% --- Tyre ---
% model = 'constant' : F_max = mu * Fz
% model = 'magic'    : simplified Magic Formula with load sensitivity
p.tire.model  = 'magic';
p.tire.mu     = 1.4;                    % used by the 'constant' model
p.tire.B      = 18.0;                   % stiffness factor
p.tire.C      = 1.65;                   % shape factor
p.tire.D      = 1.70;                   % peak longitudinal friction at zero load
p.tire.E      = 0.50;                   % curvature factor (peak force near 10 % slip)
p.tire.Dy     = 1.60;                   % peak lateral friction at zero load
p.tire.a_load = 1.2e-4;                 % load sensitivity [1/N]

% Slip ratio at which the tyre produces peak force. The simulation assumes
% an ideal traction controller that holds the tyre at this slip.
kappa = linspace(0, 0.4, 2000);
[~, idx] = max(mf_fx(kappa, 1000, p.tire));
p.tire.kappa_opt = kappa(idx);

% --- Tractive system power limit (Formula Student rules: 80 kW) ---
p.batt.enable = true;
p.batt.P_max  = 80e3;                   % [W], electrical

% --- Simulation ---
p.g            = 9.81;
p.sim.dt       = 1e-3;                  % time step [s]
p.sim.t_max    = 30;                    % abort time [s]
p.sim.d_target = 75;                    % acceleration event distance [m]
p.sim.v0       = 0.05;                  % small initial speed to avoid 0/0 [m/s]

% --- Usable torque curve (motor envelope with the power limit applied) ---
p = build_torque_curve(p);
end
